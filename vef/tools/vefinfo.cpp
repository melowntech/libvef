/**
 * Copyright (c) 2023 Melown Technologies SE
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
#include "utility/enum-io.hpp"

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

UTILITY_GENERATE_ENUM(Mode,
    ((srs))
    ((info))
)

class VefInfo : public service::Cmdline
{
public:
    VefInfo()
        : Cmdline("vefinfo", BUILD_TARGET_VERSION
                  , service::DISABLE_EXCESSIVE_LOGGING)
    {
    }

private:
    void configuration(po::options_description &cmdline
                       , po::options_description &config
                       , po::positional_options_description &pd)
        override;

    po::ext_parser extraParser() override;

    void configure(const po::variables_map &vars) override;

    bool help(std::ostream &out, const std::string &what) const override;

    int run() override;

    void info(const vef::Archive &v);

    fs::path input_;

    Mode mode_ = Mode::info;

    bool loadMeshes_ = false;
};

void VefInfo::configuration(po::options_description &cmdline
                            , po::options_description &config
                            , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("input",  po::value(&input_)->required()
         , "Path to input VEF dataset (directory, tar, zip).")
        ("mode",  po::value(&mode_)->default_value(mode_)
         , "Operation mode (info, srs).")

        ("loadMeshes"
         ,  utility::implicit_value(&loadMeshes_, true)
         ->default_value(false)
         , "No mesh is not loaded when disabled. Some information might "
         "be omitted when disabled.")
        ;

    pd
        .add("input", 1)
        ;

    (void) config;
}

po::ext_parser VefInfo::extraParser()
{
    return ([](const std::string &s) -> std::pair<std::string, std::string>
        {
            if (s == "--info") {
                return { "mode", "info" };
            }
            if (s == "--srs") {
                return { "mode", "srs" };
            }
            return {};
        });
}

void VefInfo::configure(const po::variables_map &vars)
{
    input_ = fs::absolute(input_);

    (void) vars;
}

bool VefInfo::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(vefinfo: shows VEF archive info
)RAW";
    }
    return false;
}


std::size_t submeshCount(const vef::Archive &in, const vef::Window &window)
{
    struct SubmeshCounter : geometry::ObjParserBase {
        SubmeshCounter() = default;

        std::size_t size() const { return materials_.size(); }

    private:
        void addTexture(const Vector3d&) override {}
        void addNormal(const Vector3d&) override {}
        void addFacet(const Facet&) override {}
        void materialLibrary(const std::string&) override {}
        void useMaterial(const std::string &material) override {
            materials_.insert(material);
        }
        void addVertex(const Vector3d&) override {}

        std::set<std::string> materials_;
    } loader;

    auto is(in.meshIStream(window.mesh));
    auto res(loader.parse(is->get()));
    is->close();
    if (!res) {
        LOGTHROW(err2, std::runtime_error)
            << "Unable to count submeshes in " << window.mesh.path << ".";
    }
    return loader.size();
}

void VefInfo::info(const vef::Archive &v)
{
    vef::Manifest m(v.manifest());
    for (const auto &lw : m.windows) {
        std::cout
            << "window <" << vef::name(lw) << "> @ " << lw.path << "\n"
            << "    path " << lw.path << "\n";

        if (valid(lw.extents)) {
            std::cout
                << std::fixed << "    extents " << lw.extents << "\n";
        } else {
            std::cout
                << std::fixed << "    extents invalid\n";
        }

        std::cout
            << "    lods\n"
            ;

        for (const auto &lod : lw.lods) {
            std::cout
                << "        " << lod.path << "\n"
                << "            mesh " << lod.mesh.path << "\n";

            if (loadMeshes_) {
                std::cout
                    << "                submesh count " << submeshCount(v, lod)
                    << "\n";
            }

            std::cout
                << "            atlas\n"
                ;
            for (const auto &texture : lod.atlas) {
                std::cout
                    << "                " << texture.size
                    << " @ " << texture.path << "\n"
                    ;
            }
        }
    }
}

int VefInfo::run()
{
    vef::Archive in(input_, true);

    switch (mode_) {
    case Mode::srs: {
        vef::Manifest manifest(in.manifest());
        if (!manifest.srs) {
            std::cerr << input_.string() << ": no SRS";
            return EXIT_FAILURE;
        }
        std::cout << *manifest.srs << '\n';
    } break;

    case Mode::info:
        // hic sunt leones
        info(in);
        break;
    }


    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char *argv[])
{
    return VefInfo()(argc, argv);
}
