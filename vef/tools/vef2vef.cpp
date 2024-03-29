/**
 * Copyright (c) 2021-2022 Melown Technologies SE
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
#include <optional>
#include <iostream>

#include <ogr_api.h>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filter/line.hpp>

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

#include "ogrpoly.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace ba = boost::algorithm;
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

class Vef2Vef : public service::Cmdline
{
public:
    Vef2Vef()
        : Cmdline("vef2vef", BUILD_TARGET_VERSION
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

    void convert(const vef::Archive &in, vef::ArchiveWriter &out);

    DstTrafo buildDstTrafo(const vef::Archive &in
                           , const geo::CsConvertor &conv) const;

    vef::Mesh::Format meshFormat() const {
        return (noMeshCompression_
                ? vef::Mesh::Format::obj
                : vef::Mesh::Format::gzippedObj);
    }

    fs::path input_;
    fs::path output_;
    bool overwrite_ = false;

    std::optional<geo::SrsDefinition> dstSrs_;
    std::optional<Trafo> trafo_;
    bool verticalAdjustment_ = false;
    bool noMeshCompression_ = false;
    std::optional<fs::path> clipBorder_;
    std::optional<math::Extents2> clipExtents_;

    bool flatten_ = false;
};

void Vef2Vef::configuration(po::options_description &cmdline
                            , po::options_description &config
                            , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("input",  po::value(&input_)->required()
         , "Path to input VEF dataset (directory, tar, zip).")

        ("output",  po::value(&output_)->required()
         , "Path to output VEF dataset (directory).")

        ("overwrite"
         , "Allows existing dataset overwrite if set.")

        ("dstSrs",  po::value<geo::SrsDefinition>()
         , "SRS of output dataset.")

        ("verticalAdjustment"
         ,  utility::implicit_value(&verticalAdjustment_, true)
         ->default_value(verticalAdjustment_)
         , "Apply vertical adjustment to output data."
         "Dst SRS must be a projected system to apply.")

        ("dstTrafo",  po::value<Trafo>()->implicit_value(Trafo())
         , "Forces global transformation in the output archive [sets "
         "localization transformation (global -> local)].")

        ("noMeshCompression"
         ,  utility::implicit_value(&noMeshCompression_, true)
         ->default_value(noMeshCompression_)
         , "Do not store compressed meshes.")

        ("clipBorder", po::value<fs::path>()
         , "Any vector dataset (recognizable by GDAL/ORG) that defines"
         " one or more polygons that will be used to clip the input meshes."
         " Mutually exclusive with --clipExtents.")

        ("clipExtents", po::value<math::Extents2>()
         , "Output dataset is clipped to given extents. Must be in"
         " destination SRS."
         " Mutually exclusive with --clipBorder.")

        ("flatten"
         ,  utility::implicit_value(&flatten_, true)
         ->default_value(flatten_)
         , "Flatten directory structure to just one directory per LOD.")
        ;

    pd
        .add("input", 1)
        .add("output", 1)
        ;

    (void) config;
}

void Vef2Vef::configure(const po::variables_map &vars)
{
    if (vars.count("dstSrs")) {
        dstSrs_ = vars["dstSrs"].as<geo::SrsDefinition>();
    }

    if (verticalAdjustment_) {
        if (dstSrs_ && !geo::isProjected(*dstSrs_)) {
            LOGTHROW(err2, std::runtime_error)
                << "Vertical adjustment makes sense only for "
                "projected SRS.";
        }
    }

    overwrite_ = vars.count("overwrite");

    if (vars.count("dstTrafo")) {
        trafo_ = vars["dstTrafo"].as<Trafo>();
        LOG(info3) << "Using external destination transformation: "
                   << *trafo_;
    }

    if (vars.count("clipBorder")) {
        if (vars.count("clipExtents")) {
            LOGTHROW(err2, std::runtime_error)
                << "Options --clipBorder and --clipExtents cannot be used"
                " together.";
        }
        clipBorder_ = vars["clipBorder"].as<fs::path>();
    } else if (vars.count("clipExtents")) {
        clipExtents_ = vars["clipExtents"].as<math::Extents2>();
    }
}

bool Vef2Vef::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(vef2vef: applies transformation to existing VEF dataset
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

class Convertor {
public:
    Convertor(const vef::OptionalMatrix &inTrafo = boost::none
              , const geo::CsConvertor &conv = geo::CsConvertor()
              , const geo::VerticalAdjuster &adjuster
                  = geo::VerticalAdjuster()
              , const vef::OptionalMatrix &outTrafo = boost::none)
        : inTrafo_(inTrafo)
        , conv_(conv), adjuster_(adjuster)
        , outTrafo_(outTrafo)
    {}

    template <typename Point>
    Point operator()(const Point &p) const {
        return transform(outTrafo_, adjuster_(conv_(transform(inTrafo_, p))));
    }

    template <typename Point>
    Point global(const Point &p) const {
        return adjuster_(conv_(transform(inTrafo_, p)));
    }

    template <typename Point>
    Point local(const Point &p) const {
        return transform(outTrafo_, p);
    }

private:
    vef::OptionalMatrix inTrafo_;
    geo::CsConvertor conv_;
    geo::VerticalAdjuster adjuster_;
    vef::OptionalMatrix outTrafo_;
};

math::Extents2 measure(const vef::Archive &in
                       , const vef::LoddedWindow &window
                       , const geo::CsConvertor &conv)
{
    struct Measurer : geometry::ObjParserBase {
        const Convertor conv;
        math::Extents2 extents;

        Measurer(const Convertor &conv)
            : conv(conv)
            , extents(math::InvalidExtents{})
        {}

        void addTexture(const Vector3d&) override {}
        void addNormal(const Vector3d&) override {}
        void addFacet(const Facet&) override {}
        void materialLibrary(const std::string&) override {}
        void useMaterial(const std::string&) override {}

        void addVertex(const Vector3d &v) override {
            math::update(extents, conv(math::Point3(v.x, v.y, v.z)));
        }

    } measurer(Convertor(vef::windowMatrix(in.manifest(), window), conv));

    {
        const auto &mesh(window.lods.back().mesh);

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

math::Extents2 measure(const vef::Archive &in, const geo::CsConvertor &conv)
{
    math::Extents2 extents(math::InvalidExtents{});

    if (!conv) { return extents; }

    for (const auto &window : in.manifest().windows) {
        math::update(extents, measure(in, window, conv));
    }

    return extents;
}

geo::CsConvertor makeConv(const geo::SrsDefinition &in
                          , const std::optional<geo::SrsDefinition> &out)
{
    if (out) { return { in, *out }; }
    return {};
}

struct DstTrafo {
    vef::OptionalMatrix fromGeo;
    vef::OptionalMatrix toGeo;

    DstTrafo(const vef::OptionalMatrix &fromGeo = boost::none
             , const vef::OptionalMatrix &toGeo = boost::none)
        : fromGeo(fromGeo), toGeo(toGeo)
    {}

    operator bool() const { return bool(toGeo); }
};

DstTrafo makeDstTrafo(const math::Extents2 &extents)
{
    if (!math::valid(extents)) { return {}; }
    return { geo::geo2local(extents), geo::local2geo(extents) };
}

[[maybe_unused]]
vef::OptionalMatrix makeInvDstTrafo(const math::Extents2 &extents)
{
    if (!math::valid(extents)) { return boost::none; }
    return geo::local2geo(extents);
}

class AtlasRemapping {
public:
    using optional = std::optional<AtlasRemapping>;

    AtlasRemapping(std::size_t count)
        : map_(count, -1)
    {}

    /** Assigns mapping between FROM material to TO material.
     */
    void assign(std::size_t from, std::size_t to) {
        if (from >= map_.size()) {
            LOGTHROW(err2, std::runtime_error)
                << "Invalid material remapping slot " << from << ".";
        }
        map_[from] = to;
    }

    /** Non-checking, can return -1 if mapping was not assigned.
     */
    int map(int from) const { return map_[from]; }

    /** Check for valid mapping for FROM material
     */
    bool hasMapping(int from) const { return map(from) >= 0; }

private:
    using Map = std::vector<int>;
    Map map_;
};

AtlasRemapping::optional remapAtlas(geometry::Mesh &mesh
                                    , std::size_t originalAtlasSize)
{
    // compute histogram to see what materials are used
    std::vector<std::size_t> histogram(originalAtlasSize, 0);
    for (const auto &f : mesh.faces) {
        if (f.imageId >= histogram.size()) { histogram.resize(f.imageId + 1); }
        ++histogram[f.imageId];
    }

    AtlasRemapping ar(histogram.size());

    std::size_t last(0);
    for (std::size_t id(0); id != histogram.size(); ++id) {
        if (histogram[id]) {
            // at least one face references this material;
            ar.assign(id, last++);
        }
    }

    if (last == histogram.size()) {
        // all materials are used in the mesh, no need to do any remapping
        return std::nullopt;
    }

    // we need to apply remapping to mesh faces
    for (auto &f : mesh.faces) {
        f.imageId = ar.map(f.imageId);
    }

    // ok
    return ar;
}

using FilterInit = std::function<void(bio::filtering_ostream&)>;

void copy(std::istream &is, const fs::path &filepath
          , FilterInit filterInit = {})
{
    std::ofstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    try {
        fs::create_directories(fs::absolute(filepath).parent_path());
        f.open(filepath.string(), std::ios_base::out | std::ios_base::trunc);
    } catch (const std::exception&) {
        LOGTHROW(err3, std::runtime_error)
            << "Unable to save mesh to <" << filepath << ">.";
    }

    bio::filtering_ostream fos;
    if (filterInit) { filterInit(fos); }
    fos.push(f);

    // copy data from source input stream to output
    fos << is.rdbuf();

    fos.flush();
}

void copyAtlas(const vef::Archive &in, vef::ArchiveWriter &out
               , const vef::Window &window
               , vef::Id oWindowId, vef::Id oLodId
               , const AtlasRemapping::optional &ar = std::nullopt)
{
    int tid(0);
    for (const auto &texture : window.atlas) {
        if (ar && !ar->hasMapping(tid++)) {
            // texture unused -> do not copy
            LOG(debug)
                << "Input texture " << texture.path
                << " is not used in udpated mesh, skipping.";
            continue;
        }

        auto oTexture(out.addTexture(oWindowId, oLodId
                                     , texture, texture.format));

        LOG(info3) << "Copying texture " << texture.path
                   << " to " << oTexture.path;
        {
            auto is(in.archive().istream(texture.path));
            copy(is->get(), oTexture.path);
            is->close();
        }
    }
}

class MtllibRewriter : public bio::line_filter {
public:
    std::string mtllib;
    MtllibRewriter(std::string mtllib) : mtllib(std::move(mtllib)) {}

private:
    std::string do_filter(const std::string &line) override {
        if (!ba::starts_with(line, "mtllib ")) { return line; }
        return "mtllib " + mtllib;
    };
};

inline std::string mtllib(const vef::Mesh &mesh) {
    return mesh.mtlPath().filename().generic_string();
}

void copyWindow(const vef::Archive &in, vef::ArchiveWriter &out
                , const vef::Window &window
                , vef::Id oWindowId, vef::Id oLodId)
{
    auto &oMesh(out.mesh(oWindowId, oLodId));

    LOG(info3) << "Copying mesh " << window.mesh.path
               << " to " << oMesh.path;

    {
        auto is(in.meshIStream(window.mesh));

        FilterInit finit;

        // plug-in mtllib rewriter if input and output mtllib filenames differ
        if (mtllib(window.mesh) != mtllib(oMesh)) {
            finit = [finit=std::move(finit), mtllib=mtllib(oMesh)]
                (bio::filtering_ostream &fos)
            {
                if (finit) { finit(fos); }
                fos.push(MtllibRewriter(mtllib));
            };
        }

        switch (oMesh.format) {
        case vef::Mesh::Format::obj:
            // no filter needef
            break;

        case vef::Mesh::Format::gzippedObj:
            // plug-in gzip compressor
            finit = [finit=std::move(finit)](bio::filtering_ostream &fos) {
                if (finit) { finit(fos); }
                fos.push(bio::gzip_compressor(bio::gzip_params(9), 1 << 16));
            };
            break;
        }

        // copy
        copy(is->get(), oMesh.path, finit);
        is->close();
    }

    copyAtlas(in, out, window, oWindowId, oLodId);
}

bool convertWindow(const vef::Archive &in, vef::ArchiveWriter &out
                   , const vef::Window &window
                   , vef::Id oWindowId, vef::Id oLodId
                   , const Convertor &conv
                   , const std::optional<Polygons> &clipBorder
                   , bool saveEmpty
                   , math::Extents3 &extents)
{
    auto &oMesh(out.mesh(oWindowId, oLodId));

    LOG(info3) << "Converting mesh " << window.mesh.path
               << " to " << oMesh.path;

    struct ConvertingLoader : geometry::ObjParserBase {
        const Convertor conv;
        geometry::Mesh mesh;
        std::size_t originalAtlasSize = 0;
        AtlasRemapping::optional atlasRemapping;

        unsigned int imageId = 0;

        const std::optional<Polygons> &clipBorder;

        math::Extents3 extents = math::Extents3(math::InvalidExtents{});

        ConvertingLoader(const Convertor &conv, size_t originalAtlasSize
                         , const std::optional<Polygons> &clipBorder)
            : conv(conv), originalAtlasSize(originalAtlasSize)
            , clipBorder(clipBorder)
        {}

        void addNormal(const Vector3d&) override {}
        void materialLibrary(const std::string&) override {}

        void addFacet(const Facet &f) override {
            mesh.faces.emplace_back(f.v[0], f.v[1], f.v[2]
                                    , f.t[0], f.t[1], f.t[2]
                                    , imageId);
        }

        void useMaterial(const std::string &value) override {
            imageId = boost::lexical_cast<unsigned int>(value);
        }

        void addVertex(const Vector3d &v) override {
            mesh.vertices.push_back(conv.global(math::Point3(v.x, v.y, v.z)));
        }

        void addTexture(const Vector3d &t) override {
            mesh.tCoords.emplace_back(t.x, t.y);
        }

        void finish() {
            if (clipBorder) {
                mesh = geometry::clip(mesh, *clipBorder);
                atlasRemapping = remapAtlas(mesh, originalAtlasSize);
            }

            // compute extents in destination global coordinate system
            extents = computeExtents(mesh.vertices);

            for (auto &v : mesh.vertices) {
                v = conv.local(v);
            }
        }

        bool empty() const {
            return mesh.vertices.empty();
        }

    } loader(conv, window.atlas.size(), clipBorder);

    // load and convert mesh
    {
        auto is(in.meshIStream(window.mesh));
        auto res(loader.parse(is->get()));
        loader.finish();

        is->close();
        if (!res) {
            LOGTHROW(err2, std::runtime_error)
                << "Unable to load mesh from " << window.mesh.path << ".";
        }
    }

    // do not save empty mesh if asked to
    if (!saveEmpty && loader.empty()) {
        return false;
    }

    // grab computed extents
    extents = loader.extents;

    // compute maximum absolute value of vertex coordinates
    double max(0);
    for (const auto &v : loader.mesh.vertices) {
        max = std::max({ max, std::abs(v(0))
                         , std::abs(v(1)), std::abs(v(2)) });
    }

    struct StreamSetup : geometry::ObjStreamSetup {
        bool fixed;
        StreamSetup(bool fixed) : fixed(fixed) {}

        virtual bool vertex(std::ostream &os) const {
            if (fixed) {
                os << std::fixed << std::setprecision(3);
                return true;
            }
            return false;
        }

        virtual bool tx(std::ostream &os) const {
            os.setf(std::ios::scientific, std::ios::floatfield);
            os.precision(6);
            return true;
        }
    };

    const StreamSetup streamSetup(max > 1e5);

    // make sure path exists
    fs::create_directories(fs::absolute(oMesh.path).parent_path());

    switch (oMesh.format) {
    case vef::Mesh::Format::obj:
        geometry::saveAsObj(loader.mesh, oMesh.path
                            , oMesh.mtlPath().filename().generic_string()
                            , streamSetup);
        break;

    case vef::Mesh::Format::gzippedObj:
        geometry::saveAsGzippedObj(loader.mesh, oMesh.path
                                   , oMesh.mtlPath().filename().generic_string()
                                   , streamSetup);
        break;
    }

    copyAtlas(in, out, window, oWindowId, oLodId, loader.atlasRemapping);

    return true;
}


DstTrafo Vef2Vef::buildDstTrafo(const vef::Archive &in
                                , const geo::CsConvertor &conv)
    const
{
    if (trafo_) {
        // dst trafo enforced
        return { trafo_->trafo , math::matrixInvert(trafo_->trafo) };
    }

    if (dstSrs_) {
        // localize to extents center
        return makeDstTrafo(measure(in, conv));
    }

    // no dst trafo
    return {};
}

DstTrafo chooseTrafo(const DstTrafo &t1, const vef::OptionalMatrix &t2)
{
    if (t1) { return t1; }
    if (t2) { return { math::matrixInvert(*t2), *t2 }; }
    return {};
}

bool checkExtents(const geo::CsConvertor &conv, const math::Extents3 &e3
                  , const std::optional<Polygons> &clipBorder)
{
    // clipping and valid extents? no -> cannot tell
    if (!(clipBorder && math::valid(e3))) { return true; }

    // NxM segments, (N+1)x(M+1) vertices
    const math::Size2 grid(10, 10);

    geometry::Mesh mesh;

    const auto extents(math::extents2(e3));

    const auto origin(extents.ll);
    auto size(math::size(extents));

    size.width /= grid.width;
    size.height /= grid.height;

    // generate vertex grid in destination SRS
    {
        math::Point3 p(0, 0, math::center(e3)(2));
        for (int j(0); j <= grid.height; ++j) {
            p(1) = origin(1) + size.height * j;
            for (int i(0); i <= grid.width; ++i) {
                p(0) = origin(0) + size.width * i;
                mesh.vertices.push_back(conv(p));
            }
        }
    }

    // generate faces
    {
        const int w(grid.width + 1);
        for (int j(0); j < grid.height; ++j) {
            int v(w * j);
            for (int i(0); i < grid.width; ++i, ++v) {
                mesh.faces.emplace_back(v, v + 1, v + w);
                mesh.faces.emplace_back(v + w, v + 1, v + w + 1);
            }
        }
    }

    // try to clip
    const auto clipped(geometry::clip(mesh, *clipBorder));

    // valid only if there was anything inside the clip region
    return !clipped.faces.empty();
}

void Vef2Vef::convert(const vef::Archive &in, vef::ArchiveWriter &out)
{
    const auto clipBorder
        (clipBorder_
         ? loadPolygons(clipBorder_, out.getSrs().value())
         : polygonsFromExtents(clipExtents_));

    const auto geoConv(makeConv(*in.manifest().srs, dstSrs_));
    const auto dstTrafo(buildDstTrafo(in, geoConv));

    // use new trafo or reuse existing?
    const auto useDstTrafo(chooseTrafo(dstTrafo, in.manifest().trafo));

    out.setTrafo(useDstTrafo.toGeo);

    // windows to delete -- we cannot delete windows due to OpenMP
    vef::Ids windowsToDelete;

    const auto &iManifest(in.manifest());

    // make room for input windows so we need only synchronize window allocation
    out.expectWindows(iManifest.windows.size());

    UTILITY_OMP(parallel)
    {
        const auto gc(geoConv.clone());

        const geo::VerticalAdjuster verticalAdjuster
            (dstSrs_
             ? geo::VerticalAdjuster(verticalAdjustment_, *dstSrs_)
             : geo::VerticalAdjuster());

        UTILITY_OMP(for schedule(dynamic))
        for (std::size_t wi = 0; wi < iManifest.windows.size(); ++wi) {
            const auto &iWindow(iManifest.windows[wi]);

            Convertor conv(vef::windowMatrix(in.manifest(), iWindow)
                           , gc, verticalAdjuster, useDstTrafo.fromGeo);

            vef::OptionalString name(iWindow.name);
            if (!name) {
                name = fs::path(iWindow.path).filename().generic_string();
            }

            // check window extents clipping first
            if (!checkExtents(gc, iWindow.extents, clipBorder)){
                LOG(info3)
                    << "Skipping empty window <" << name.value()
                    << "> (based on window extents)";
                continue;
            }

            const auto oWindowId([&]()
            {
                vef::OptionalMatrix trafo;
                if (!dstTrafo) {
                    // no DST trafo set, use original trafo
                    trafo = iWindow.trafo;
                }

                vef::Id id;
                UTILITY_OMP(critical(vef2vef_convert_addWindow))
                    id = out.addWindow(boost::none, trafo, name);
                return id;
            }());

            bool first(true);
            bool abandon(false);
            math::Extents3 combinedExtents(math::InvalidExtents{});
            for (const auto &iLod : iWindow.lods) {
                vef::Id oLodId;
                oLodId = out.addLod(oWindowId, boost::none, meshFormat());

                math::Extents3 extents;

                const bool mustConvert(!math::valid(iWindow.extents)
                                       || dstSrs_
                                       || dstTrafo
                                       || clipBorder);

                if (mustConvert) {
                    if (!convertWindow(in, out, iLod, oWindowId, oLodId, conv
                                       , clipBorder, !first, extents))
                    {
                        abandon = true;
                        break;
                    }
                } else {
                    // just reuse extents
                    extents = iWindow.extents;
                    copyWindow(in, out, iLod, oWindowId, oLodId);
                }

                // update whole window extents
                math::update(combinedExtents, extents);

                first = false;
            }

            if (abandon) {
                LOG(info3)
                    << "Skipping empty window <" << name.value() << ">";
                UTILITY_OMP(critical(vef2vef_convert_deleteWindow))
                    windowsToDelete.push_back(oWindowId);
            } else {
                out.setExtents(oWindowId, combinedExtents);
            }
        }
    }

    std::sort(windowsToDelete.begin(), windowsToDelete.end()
              , std::greater<vef::Id>());

    for (const auto windowId : windowsToDelete) {
        out.deleteWindow(windowId);
    }
}

int Vef2Vef::run()
{
    vef::Archive in(input_);

    if (dstSrs_ && !in.manifest().srs) {
        std::cerr << "Input VEF dataset has no SRS, cannot transform."
                  << std::endl;
        return EXIT_FAILURE;
    }

    vef::ArchiveWriter out(output_, overwrite_, flatten_);

    if (dstSrs_) {
        out.setSrs(*dstSrs_);
    } else if (in.manifest().srs) {
        out.setSrs(*in.manifest().srs);
    }

    convert(in, out);

    out.flush();

    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char *argv[])
{
    ::OGRRegisterAll();
    return Vef2Vef()(argc, argv);
}
