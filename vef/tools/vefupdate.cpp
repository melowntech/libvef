/**
 * Copyright (c) 2022 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cstdlib>
#include <string>
#include <iostream>

#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/optional/optional_io.hpp>

#include "dbglog/dbglog.hpp"

#include "service/cmdline.hpp"
#include "utility/buildsys.hpp"
#include "utility/implicit-value.hpp"
#include "utility/openmp.hpp"
#include "utility/format.hpp"

#include "math/transform.hpp"

#include "geometry/mesh.hpp"
#include "geometry/meshop.hpp"
#include "geometry/parse-obj.hpp"

#include "geo/csconvertor.hpp"
#include "geo/coordinates.hpp"
#include "geo/verticaladjuster.hpp"

#include "vef/vef.hpp"
#include "vef/reader.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace bio = boost::iostreams;

namespace {

struct Trafo {
    math::Matrix4 trafo;

    Trafo() : trafo(math::identity4()) {}
};

std::ostream& operator<<(std::ostream &os, const Trafo &t)
{
    std::string sep("");
    for (int j(0); j < 3; ++j) {
        for (int i(0); i < 4; ++i) {
            os << sep << t.trafo(j, i);
            sep = ",";
        }
    }
    return os;
}

struct DstTrafo;
class Convertor;

std::istream& operator>>(std::istream &is, Trafo &t)
{
    t.trafo = math::identity4();

    bool comma(false);
    for (int j(0); j < 3; ++j) {
        for (int i(0); i < 4; ++i) {
            if (comma) { is >> utility::expect(','); }
            comma = true;
            is >> t.trafo(j, i);
        }
    }
    return is;
}

class VefUpdate : public service::Cmdline
{
public:
    VefUpdate()
        : Cmdline("vefupdate", BUILD_TARGET_VERSION
                  , service::DISABLE_EXCESSIVE_LOGGING)
    {
    }

private:
    virtual void configuration(po::options_description &cmdline
                               , po::options_description &config
                               , po::positional_options_description &pd)
        override;

    virtual void configure(const po::variables_map &vars)
        override;

    virtual bool help(std::ostream &out, const std::string &what)
        const override;

    virtual int run() override;

    fs::path input_;
    fs::path output_;

    int depth_ = 0;
    std::set<std::size_t> lods_;
};

void VefUpdate::configuration(po::options_description &cmdline
                            , po::options_description &config
                            , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("input",  po::value(&input_)->required()
         , "Path to input VEF dataset (directory, tar, zip).")

        ("output", po::value(&output_)->required()
         , "Path to output VEF manifest file.")

        ("depth", po::value<int>()
         , "Limit output only to given depth from fines LODs (>0)"
         "or coarsest LODs (<0). 0 means keep all windows as they are.")

        ("lod", po::value<std::vector<std::size_t>>()
         , "Cherry pick individual output LODs.")
        ;

    pd
        .add("input", 1)
        .add("output", 1)
        ;

    (void) config;
}

void VefUpdate::configure(const po::variables_map &vars)
{
    input_ = fs::absolute(input_);
    output_ = fs::absolute(output_);

    if (vars.count("lod")) {
        if (vars.count("depth")) {
            throw std::logic_error
                ("Options --lod and --depth cannot be used together.");
        }
        const auto &lods(vars["lod"].as<std::vector<std::size_t>>());
        lods_.insert(lods.begin(), lods.end());
    }

    if (vars.count("depth")) {
        depth_ = vars["depth"].as<int>();
    }
}

bool VefUpdate::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(vefupdates: generates updated VEF manifest to latest spec
)RAW";
    }
    return false;
}


template <typename Point>
Point transform(const vef::OptionalMatrix &trafo
                , const Point &p)
{
    if (trafo) { return math::transform(*trafo, p); }
    return p;
}

math::Extents3 measure(const vef::Archive &in
                       , const vef::LoddedWindow &window)
{
    struct Measurer : geometry::ObjParserBase {
        const vef::OptionalMatrix trafo;
        math::Extents3 extents;

        Measurer(const vef::OptionalMatrix &trafo)
            : trafo(trafo)
            , extents(math::InvalidExtents{})
        {}

        void addTexture(const Vector3d&) override {}
        void addNormal(const Vector3d&) override {}
        void addFacet(const Facet&) override {}
        void materialLibrary(const std::string&) override {}
        void useMaterial(const std::string&) override {}

        void addVertex(const Vector3d &v) override {
            math::update
                (extents, transform(trafo, math::Point3(v.x, v.y, v.z)));
        }

    } measurer(vef::windowMatrix(in.manifest(), window));

    LOG(info3) << "Measuring window "
               << window.name.value_or(window.path.generic_string())
               << ".";

    for (const auto &lod : window.lods) {
        LOG(info2) << "Measuring lod " << lod.path << ".";
        const auto &mesh(lod.mesh);

        auto is(in.meshIStream(mesh));
        auto res(measurer.parse(is->get()));
        is->close();
        if (!res) {
            LOGTHROW(err2, std::runtime_error)
                << "Unable to measure mesh from " << mesh.path << ".";
        }
    }

    return measurer.extents;
}

int VefUpdate::run()
{
    vef::Archive in(input_, false);

    vef::Manifest manifest(in.manifest());

    int count(manifest.windows.size());

    UTILITY_OMP(parallel for schedule(dynamic, 1))
    for (int i = 0; i < count; ++i) {
        auto &lw(manifest.windows[i]);
        if (!lw.name) { lw.name = lw.path.filename().generic_string(); }
        lw.extents = measure(in, lw);

        if (!lods_.empty()) {
            vef::Window::list lods;
            for (auto lod : lods_) {
                if (lod < lw.lods.size()) {
                    lods.push_back(lw.lods[lod]);
                }
            }
            std::swap(lw.lods, lods);
        } else if (depth_ > 0) {
            const std::size_t depth(depth_);
            if (lw.lods.size() > depth) {
                // NB: using first element as value to allow resizing vector of
                // non-default-constructable type... *sigh*
                // NB: it's never used, since we are reducing the vector, not
                // enlarging
                lw.lods.resize(depth, lw.lods.front());
            }
        } else if (depth_ < 0) {
            const std::size_t depth(-depth_);
            if (lw.lods.size() > depth) {
                lw.lods.erase(lw.lods.begin(), lw.lods.end() - depth);
            }
        }
    }

    // TODO: handle non-directory based paths
    vef::saveManifest(output_, manifest, input_);

    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char *argv[])
{
    return VefUpdate()(argc, argv);
}
