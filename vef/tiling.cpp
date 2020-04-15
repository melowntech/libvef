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

#include "geo/csconvertor.hpp"
#include "geo/enu.hpp"
#include "geo/po.hpp"

#include "tiling.hpp"

namespace vef {

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
    math::Extents2 extents;

    MeshInfo() : extents(math::InvalidExtents{}) {}

    void update(const MeshInfo &mi) {
        area.mesh += mi.area.mesh;
        area.submeshes.insert(area.submeshes.end(), mi.area.submeshes.begin()
                              , mi.area.submeshes.end());
        math::update(extents, mi.extents);
    }
};

/** Measures whole mesh extents from coarsest data.
 */
math::Extents3 meshExtents(const Archive &archive)
{
    struct ExtentsMeasurer : public geometry::ObjParserBase {
        ExtentsMeasurer() : extents(math::InvalidExtents{}) {}

        virtual void addVertex(const Vector3d &v) {
            math::update(extents, math::Point3d(v));
        }

        virtual void addTexture(const Vector3d&) {}
        virtual void addNormal(const Vector3d&) {}
        virtual void addFacet(const Facet&) {}
        virtual void materialLibrary(const std::string&) {}
        virtual void useMaterial(const std::string&) {}

        math::Extents3 extents;
    };

    ExtentsMeasurer em;

    for (const auto &lw : archive.manifest().windows) {
        const auto &window(lw.lods.back());
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
                     , const geo::CsConvertor &conv)
{
    LOG(info2) << "Loading mesh from " << mesh.path << ".";

    class MeshMeasurer : public geometry::ObjParserBase {
    public:
        MeshMeasurer(const geo::CsConvertor &conv)
            : conv_(conv)
        {
            useMaterial(0);
        }

        virtual void addVertex(const Vector3d &v) {
            vertices_.push_back(conv_(math::Point3(v.x, v.y, v.z)));
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
                math::update(mi.extents, v(0), v(1));
            }
            return mi;
        }

        virtual void addNormal(const Vector3d&) { /*ignored*/ }
        virtual void materialLibrary(const std::string&) { /*ignored*/ }

    private:
        const geo::CsConvertor &conv_;

        math::Points3 vertices_;
        math::Points2 tc_;
        SubMeshArea::list submeshes_;
        unsigned int textureId_;
    };

    MeshMeasurer mm(conv);
    if (!mm.parse(*archive.meshIStream(mesh))) {
        LOGTHROW(err2, std::runtime_error)
            << "Unable to load mesh from OBJ file at " << mesh.path << ".";
    }
    return mm.info();
}

MeshInfo measureMeshes(const Archive &archive
                       , const geo::CsConvertor &conv)
{
    MeshInfo mi;

    const auto &windows(archive.manifest().windows);

    UTILITY_OMP(parallel for)
    for (std::size_t i = 0; i < windows.size(); ++i) {
        const auto &window(windows[i].lods.front());

        auto a(measureMesh(archive, window.mesh, conv));

        // expand texture area
        auto iatlas(window.atlas.begin());
        for (auto &sm : a.area.submeshes) {
            // expand by texture size
            const auto &size((*iatlas++).size);
            sm.texture *= size.width;
            sm.texture *= size.height;
        }

        UTILITY_OMP(critical(vef23dtiles_measureMeshes_1))
            mi.update(a);
    }
    return mi;
}

struct MeshParams {
    math::Extents2 extents;
    geo::SrsDefinition workSrs;
    double pixelArea;

    MeshParams() : pixelArea() {}
};

inline double pixelArea(const MeshArea &area)
{
    double ta(0);
    for (const auto &sm : area.submeshes) {
        ta += sm.texture;
    }
    return area.mesh / ta;
}

MeshParams analyzeMesh(const Archive &archive)
{
    const auto &srcSrs(*archive.manifest().srs);

    if (srcSrs.is(geo::SrsDefinition::Type::enu)) {
        // fine, it's ENU -> measure mesh in src/work SRS
        const auto mi(measureMeshes(archive, geo::CsConvertor()));

        MeshParams mp;
        mp.extents = mi.extents;
        mp.workSrs = srcSrs;
        mp.pixelArea = pixelArea(mi.area);
        return mp;
    }

    // not ENU, build one

    // get center of mesh in its source SRS
    const auto center(math::center(meshExtents(archive)));

    // build ENU
    // TODO: extract spheroid and towgs84 from srsSrs
    geo::Enu enu(geo::CsConvertor(srcSrs, srcSrs.geographic())(center));

    MeshParams mp;
    mp.workSrs = geo::SrsDefinition::fromEnu(enu);

    // measure mesh in work SRS
    const auto mi(measureMeshes(archive, geo::CsConvertor(srcSrs, enu)));
    mp.extents = mi.extents;
    mp.pixelArea = pixelArea(mi.area);

    return mp;
}

math::Extents2 makeExtents(const math::Point2 &center
                           , const math::Size2f &size)
{
    // measure extents
    return { center(0) - size.width / 2.0
            , center(1) - size.height / 2.0
            , center(0) + size.width  / 2.0
            , center(1) + size.height  / 2.0 };
}

} // namespace

Tiling::Tiling(const Archive &archive, const math::Size2 &optimalTextureSize)
    : srcSrs(*archive.manifest().srs), maxLod()
{
    const auto mp(analyzeMesh(archive));

    workSrs = mp.workSrs;

    int depth(0);
    for (const auto &lw : archive.manifest().windows) {
        const int nd(lw.lods.size());
        if (nd > depth) { depth = nd; }
    }

    auto lodDiff(depth - 1);

    // compute area of one pixel (meter^2/pixel)
    const auto pxSize(std::sqrt(mp.pixelArea));

    const math::Size2f minTileSize
        (pxSize * optimalTextureSize.width
         , pxSize * optimalTextureSize.height);

    const math::Size2f maxTileSize
        (minTileSize.width * (1 << lodDiff)
         , minTileSize.height * (1 << lodDiff));

    const auto sceneSize(math::size(mp.extents));

    // size of scene in max-tiles (square)
    auto sizeInTiles
        (std::max(std::ceil(sceneSize.width / maxTileSize.width)
                  , std::ceil(sceneSize.height / maxTileSize.height)));

    // number of lods needed to add above scene top lod
    const int headLods(std::ceil(std::log2(sizeInTiles)));

    const math::Size2f worldSize
        ((1 << headLods) * maxTileSize.width
         , (1 << headLods) * maxTileSize.height);

    workExtents = makeExtents(math::center(mp.extents), worldSize);

    maxLod = lodDiff + headLods;

    LOG(info3)
        << std::fixed
        << "Tiling: maxLod: " << maxLod
        << ", empty lods: " << headLods
        << ", extents: " << workExtents
        << "; calculated from pixel size: " << pxSize
        << ", VEF archive depth: " << depth
        << ", and mesh extents: " << mp.extents
        << ".";
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
