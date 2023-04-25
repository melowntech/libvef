/**
 * Copyright (c) 2018 Melown Technologies SE
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

#include "dbglog/dbglog.hpp"
#include "utility/openmp.hpp"
#include "geometry/parse-obj.hpp"
#include "math/geometry.hpp"
#include "math/transform.hpp"
#include "math/math.hpp"

#include "geo/csconvertor.hpp"
#include "geo/enu.hpp"
#include "geo/po.hpp"

#include "tiling.hpp"

namespace vef {

namespace {

math::Extents3 makeFullWorldExtents(const math::Extents2 &e2)
{
    auto size(math::size(e2));
    auto halfLength((size.width + size.height) / 4);

    return { e2.ll(0), e2.ll(1), -halfLength
             , e2.ur(0), e2.ur(1), halfLength };
}

} // namespace

World::World(const geo::SrsDefinition &srs, const math::Extents2 &extents)
    : srs(srs), extents(makeFullWorldExtents(extents))
{}

namespace {

/** Single submesh area
 */
struct SubMeshArea {
    double mesh;
    double texture;

    typedef std::vector<SubMeshArea> list;

    SubMeshArea() : mesh(), texture() {}
};

/** Whole mesh area
 */
struct MeshArea {
    double mesh;
    SubMeshArea::list submeshes;

    MeshArea() : mesh() {}
};

struct MeshInfo {
    MeshArea area;
    math::Extents3 extents;

    MeshInfo() : extents(math::InvalidExtents{}) {}

    void update(const MeshInfo &mi) {
        area.mesh += mi.area.mesh;
        area.submeshes.insert(area.submeshes.end(), mi.area.submeshes.begin()
                              , mi.area.submeshes.end());
        math::update(extents, mi.extents);
    }
};

math::Point3 optionalTransform(const OptionalMatrix &trafo
                               , const math::Point3 &p)
{
    if (!trafo) { return p; }
    return math::transform(*trafo, p);
}

/** Measures whole mesh extents from coarsest data.
 */
math::Extents3 meshExtents(const Archive &archive)
{
    struct ExtentsMeasurer : public geometry::ObjParserBase {
        ExtentsMeasurer()
            : extents(math::InvalidExtents{}) {}

        virtual void addVertex(const Vector3d &v) {
            math::update(extents, optionalTransform(trafo, math::Point3d(v)));
        }

        virtual void addTexture(const Vector3d&) {}
        virtual void addNormal(const Vector3d&) {}
        virtual void addFacet(const Facet&) {}
        virtual void materialLibrary(const std::string&) {}
        virtual void useMaterial(const std::string&) {}

        OptionalMatrix trafo;
        math::Extents3 extents;
    };

    ExtentsMeasurer em;

    for (const auto &lw : archive.manifest().windows) {
        const auto &window(lw.lods.back());
        em.trafo = windowMatrix(archive.manifest(), lw);
        if (!em.parse(*archive.meshIStream(window.mesh))) {
            LOGTHROW(err2, std::runtime_error)
                << "Unable to load mesh from OBJ file at "
                << window.mesh.path << ".";
        }
    }

    return em.extents;
}

inline double triangleArea(const math::Point3 &a, const math::Point3 &b,
                           const math::Point3 &c)
{
    return norm_2(math::crossProduct(b - a, c - a)) / 2.0;
}

inline double triangleArea(const math::Point2 &a, const math::Point2 &b,
                           const math::Point2 &c)
{
    return std::abs
        (math::crossProduct(math::Point2(b - a), math::Point2(c - a)))
        / 2.0;
}

MeshInfo measureMesh(const Archive &archive, const Mesh &mesh
                     , const OptionalMatrix &trafo
                     , const geo::CsConvertor &conv)
{
    LOG(info2) << "Loading mesh from " << mesh.path << ".";

    class MeshMeasurer : public geometry::ObjParserBase {
    public:
        MeshMeasurer(const OptionalMatrix &trafo, const geo::CsConvertor &conv)
            : trafo_(trafo), conv_(conv)
        {
            useMaterial(0);
        }

        virtual void addVertex(const Vector3d &v) {
            vertices_.push_back
                (conv_(optionalTransform(trafo_, math::Point3(v))));
        }

        virtual void addTexture(const Vector3d &t) {
            tc_.emplace_back(t.x, t.y);
        }

        virtual void addFacet(const Facet &f) {
            auto &sm(submeshes_[textureId_]);
            sm.mesh += triangleArea
                (vertices_[f.v[0]], vertices_[f.v[1]], vertices_[f.v[2]]);
            sm.texture += triangleArea(tc_[f.t[0]], tc_[f.t[1]], tc_[f.t[2]]);
        }

        virtual void useMaterial(const std::string &m) {
            // get new material index
            try {
                useMaterial(boost::lexical_cast<unsigned int>(m));
            } catch (const boost::bad_lexical_cast &e) {
                LOGTHROW(err2, std::runtime_error)
                    << "Not a numberic material: <" << m << ">.";
            }
        }

        void useMaterial(unsigned int textureId) {
            textureId_ = textureId;
            if (submeshes_.size() <= textureId_) {
                submeshes_.resize(textureId_ + 1);
            }
        }

        MeshInfo info() const {
            MeshInfo mi;
            mi.area.submeshes = submeshes_;
            for (const auto &sm : mi.area.submeshes) {
                mi.area.mesh += sm.mesh;
            }

            for (const auto &v : vertices_) {
                math::update(mi.extents, v);
            }
            return mi;
        }

        virtual void addNormal(const Vector3d&) { /*ignored*/ }
        virtual void materialLibrary(const std::string&) { /*ignored*/ }

    private:
        const OptionalMatrix &trafo_;
        const geo::CsConvertor &conv_;

        math::Points3 vertices_;
        math::Points2 tc_;
        SubMeshArea::list submeshes_;
        unsigned int textureId_;
    };

    MeshMeasurer mm(trafo, conv);
    if (!mm.parse(*archive.meshIStream(mesh))) {
        LOGTHROW(err2, std::runtime_error)
            << "Unable to load mesh from OBJ file at " << mesh.path << ".";
    }
    return mm.info();
}

struct Done {
    Done(std::size_t count, std::size_t total)
        : count(count), total(total)
    {}

    std::size_t count;
    std::size_t total;
};

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const Done &d)
{
    if (d.total) {
        double percentage((100.0 * d.count) / d.total);
        boost::io::ios_precision_saver ps(os);
        return os << '#' << d.count << " of " << d.total << " ("
                  << std::fixed << std::setprecision(2)
                  << std::setw(6) << percentage
                  << " % done)";
    }
    return os << '#' << d.count;
}

MeshInfo measureMeshes(const Archive &archive
                       , const geo::CsConvertor &inConv = geo::CsConvertor())
{
    MeshInfo mi;

    const auto &windows(archive.manifest().windows);

    //std::atomic<std::size_t> measured(0);

    // OpenMP magic: create an empty CS conv and tell OpenMP to create a private
    // instance for each thread
    // geo::CsConvertor conv;
    UTILITY_OMP(parallel)
    {
        // clone input cs to the local cs conv
        geo::CsConvertor conv = inConv.clone();

        UTILITY_OMP(for)
        for (int i = 0; i < static_cast<int>(windows.size()); ++i) {
        // for (std::size_t i = 0; i < windows.size(); ++i) {
            const auto &lWindows(windows[i]);
            const auto &window(lWindows.lods.front());

            auto a(measureMesh(archive, window.mesh
                               , windowMatrix(archive.manifest(), lWindows)
                               , conv));

            // expand texture area
            auto iatlas(window.atlas.begin());
            for (auto &sm : a.area.submeshes) {
                // expand by texture size
                const auto &size((*iatlas++).size);
                sm.texture *= size.width;
                sm.texture *= size.height;
            }

            UTILITY_OMP(critical(vef_tiling_measureMeshes_1))
                mi.update(a);

            //Done done(++measured, windows.size());
            //LOG(info3) << "Measured " << done << " meshes.";
        }
    }
    return mi;
}

struct MeshParams {
    math::Extents3 extents;
    geo::SrsDefinition workSrs;
    double pixelArea;

    MeshParams() : pixelArea() {}
};

inline double pixelArea(const MeshArea &area
                        , const boost::optional<double> &resolution
                        = boost::none)
{
    // override
    if (resolution) { return math::sqr(resolution.value()); }

    double ta(0);
    for (const auto &sm : area.submeshes) {
        ta += sm.texture;
    }
    return area.mesh / ta;
}

inline bool compatible(const geo::SrsDefinition &srs)
{
    return (geo::isProjected(srs) || srs.is(geo::SrsDefinition::Type::enu));
}

MeshParams
analyzeMesh(const Archive &archive
            , const boost::optional<double> &resolution = boost::none)
{
    const auto &srs(archive.manifest().srs.value());

    if (compatible(srs)) {
        // work in given SRS
        const auto mi(measureMeshes(archive));

        MeshParams mp;
        mp.extents = mi.extents;
        mp.workSrs = srs;
        mp.pixelArea = pixelArea(mi.area, resolution);
        return mp;
    }

    // build ENU

    // TODO: extract spheroid and towgs84 from srs
    const auto center(math::center(meshExtents(archive)));
    geo::Enu enu(geo::CsConvertor(srs, srs.geographic())(center));

    MeshParams mp;
    mp.workSrs = geo::SrsDefinition::fromEnu(enu);

    // measure mesh in work SRS
    const auto mi(measureMeshes(archive, geo::CsConvertor(srs, enu)));
    mp.extents = mi.extents;
    mp.pixelArea = pixelArea(mi.area);

    return mp;
}

MeshParams
analyzeMesh(const Archives &archives
            , const boost::optional<double> &resolution = boost::none)
{
    if (archives.size() == 1) {
        // single archive
        return analyzeMesh(archives.front(), resolution);
    }

    // multiple archives
    geo::SrsDefinition srs(archives.front().get().manifest().srs.value());
    bool useSrs(compatible(srs));

    if (useSrs) {
        for (const Archive &archive : archives) {
            geo::SrsDefinition asrs(archive.manifest().srs.value());
            if (!geo::areSame(srs, asrs)) {
                // srs is not compatible or is not same as the first SRS
                useSrs = false;
                break;
            }
        }
    }

    MeshParams mp;

    if (useSrs) {
        // work in given SRS
        MeshInfo mi;
        // combine all meshes
        for (const Archive &archive : archives) {
            const auto ami(measureMeshes(archive));
            mi.update(ami);
        }

        mp.extents = mi.extents;
        mp.workSrs = srs;
        mp.pixelArea = pixelArea(mi.area, resolution);
        return mp;
    } else {
        // TODO: use common ENU
        LOGTHROW(err3, std::runtime_error)
            << "Support for different SRS not implemented, yet.";
    }

    return mp;
}

MeshParams
analyzeMesh(const Archives &archives
            , const geo::SrsDefinition &srs
            , const boost::optional<double> &resolution = boost::none)
{
    // work in given SRS
    MeshInfo mi;
    // combine all meshes
    for (const Archive &archive : archives) {
        const auto &inSrs(archive.manifest().srs.value());
        const auto ami(measureMeshes
                       (archive, geo::CsConvertor(inSrs, srs)));
        mi.update(ami);
    }

    MeshParams mp;
    mp.extents = mi.extents;
    mp.workSrs = srs;
    mp.pixelArea = pixelArea(mi.area, resolution);
    return mp;
}

math::Extents3 makeExtents(const math::Point3 &center
                           , const math::Size3f &size)
{
    // measure extents
    return { center(0) - size.width / 2.0
             , center(1) - size.height / 2.0
             , center(2) - size.depth / 2.0
             , center(0) + size.width / 2.0
             , center(1) + size.height / 2.0
             , center(2) + size.depth / 2.0 };
}

} // namespace

Tiling::Tiling(const Archives &archives
               , const math::Size2 &optimalTextureSize
               , bool for3dCutting
               , const boost::optional<double> &resolution
               , const boost::optional<World> &world
               , std::size_t extraLods)
    : maxLod()
{
    // sanity checks
    if (archives.empty()) {
        LOGTHROW(err2, std::runtime_error)
            << "Tiling needs at least one VEF archive to work with.";
    }

    for (const Archive &archive : archives) {
        if (!archive.manifest().srs) {
            LOGTHROW(err2, std::runtime_error)
                << "Archive " << archive.path() << " is not georeferenced "
                "(missing SRS field).";
        }
    }

    int depth(0);
    for (const Archive &archive : archives) {
        for (const auto &lw : archive.manifest().windows) {
            const int nd(lw.lods.size());
            if (nd > depth) { depth = nd; }
        }
    }

    depth += extraLods;

    auto lodDiff(depth - 1);

    if (!world || !math::valid(world->extents)) {
        const auto mp(world
                      ? analyzeMesh(archives, world->srs, resolution)
                      : analyzeMesh(archives, resolution));

        workSrs = mp.workSrs;

        // compute area of one pixel (meter^2/pixel)
        const auto pxSize(std::sqrt(mp.pixelArea));

        const math::Size2f minTileSize
            (pxSize * optimalTextureSize.width
             , pxSize * optimalTextureSize.height);

        const math::Size2f maxTileSize
            (minTileSize.width * (1 << lodDiff)
             , minTileSize.height * (1 << lodDiff));

        const auto sceneSize(math::size(mp.extents));

        // max tile depth, average of width and height
        auto maxTileDepth((maxTileSize.width + maxTileSize.height) / 2.0);

        // size of scene in max-tiles (square)
        auto sizeInTiles
            (std::max(std::ceil(sceneSize.width / maxTileSize.width)
                      , std::ceil(sceneSize.height / maxTileSize.height)));

        if (for3dCutting) {
            sizeInTiles = std::max
                (sizeInTiles, std::ceil(sceneSize.depth / maxTileDepth));
        }

        // number of lods needed to add above scene top lod
        const int headLods(std::ceil(std::log2(sizeInTiles)));

        math::Size3f worldSize
            ((1 << headLods) * maxTileSize.width
             , (1 << headLods) * maxTileSize.height
             , (1 << headLods) * maxTileDepth);

        workExtents = makeExtents(math::center(mp.extents), worldSize);

        maxLod = lodDiff + headLods;

        // store pixel size as resolution
        this->resolution = pxSize;

        LOG(info3)
            << std::fixed
            << "Tiling: maxLod: " << maxLod
            << ", empty lods: " << headLods
            << ", extents: " << workExtents
            << "; calculated from pixel size: " << pxSize
            << ", VEF archive depth: " << depth
            << ", and mesh extents: " << mp.extents
            << ".";

    } else {
        workSrs = world->srs;

        // analyze in provided SRS
        const auto mp(analyzeMesh(archives, workSrs, resolution));

        workExtents = world->extents;

        this->resolution = std::sqrt(mp.pixelArea);

        auto optimalTileArea(area(optimalTextureSize) * mp.pixelArea);
        const auto optimalTileCount(extents2(workExtents).area()
                                    / optimalTileArea);

        // find best LOD
        maxLod = 0.5 * std::log2(optimalTileCount);

        LOG(info3)
            << std::fixed
            << "Tiling (using provided World definition): maxLod: " << maxLod
            << ", extents: " << workExtents
            << "; calculated from pixel size: " << this->resolution
            << ".";
    }
}

Tiling::Tiling(std::ostream &os)
{
    (void) os;
}

void Tiling::save(std::ostream &os) const
{
    (void) os;
}

} // namespace vef
