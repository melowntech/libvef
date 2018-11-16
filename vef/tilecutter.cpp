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

#include <opencv2/highgui/highgui.hpp>

#include "utility/openmp.hpp"

#include "vts-libs/vts/meshop.hpp"
#include "vts-libs/tools-support/repackatlas.hpp"

#include "./tilecutter.hpp"

namespace fs = boost::filesystem;

namespace vts = vtslibs::vts;
namespace tools = vtslibs::vts::tools;

namespace vef {

namespace {

/** Per-window record to ease parallel processing.
 */
struct WindowRecord {
    const vef::Window *window;
    vts::Lod lod;

    WindowRecord(const vef::Window &window, vts::Lod lod)
        : window(&window), lod(lod)
    {}

    typedef std::vector<WindowRecord> list;
};

WindowRecord::list windowRecordList(const vef::Archive &archive
                                    , vts::Lod maxLod)
{
    WindowRecord::list list;

    for (const auto &lw : archive.manifest().windows) {
        auto lod(maxLod);
        for (const auto &w : lw.lods) {
            list.emplace_back(w, lod--);
        }
    }

    return list;
}

inline void warpInPlace(vts::SubMesh &mesh, const geo::CsConvertor &conv)
{
    for (auto &v : mesh.vertices) { v = conv(v); }
}

inline void warpInPlace(vts::Mesh &mesh, const geo::CsConvertor &conv)
{
    for (auto &sm : mesh) { warpInPlace(sm, conv); }
}

class Cutter {
public:
    Cutter(tools::TmpTileset &ts, const vef::Archive &archive
           , const math::Extents2 &worldExtents
           , const geo::CsConvertor &vef2world
           , int maxLod, double clipMargin)
        : ts_(ts), archive_(archive)
        , worldExtents_(worldExtents)
        , vef2world_(vef2world)
        , clipMargin_(clipMargin)
        , windows_(windowRecordList(archive_, maxLod))
    {}

    void operator()(/**vt::ExternalProgress &progress*/);

private:
    void windowCut(const WindowRecord &window);

    void splitToTiles(vts::Lod lod, const vts::TileRange &tr
                      , const vts::Mesh &mesh
                      , const vts::opencv::Atlas &atlas);

    void tileCut(const vts::TileId &tileId, const vts::Mesh &mesh
                 , const vts::opencv::Atlas &atlas);

    cv::Mat loadTexture(const fs::path &path) const;

    tools::TmpTileset &ts_;
    const vef::Archive &archive_;
    const math::Extents2 &worldExtents_;
    const geo::CsConvertor &vef2world_;
    const  double clipMargin_;
    WindowRecord::list windows_;
};

void Cutter::operator()(/**vt::ExternalProgress &progress*/)
{
    UTILITY_OMP(parallel for)
        for (std::size_t i = 0; i < windows_.size(); ++i) {
            windowCut(windows_[i]);
        }

    ts_.flush();
}

cv::Mat Cutter::loadTexture(const fs::path &path) const
{
    const auto &archive(archive_.archive());
    if (archive.directio()) {
        // optimized access
        auto tex(cv::imread(archive.path(path).string()));
        if (!tex.data) {
            LOGTHROW(err2, std::runtime_error)
                << "Unable to load texture from " << path << ".";
        }
    }

    auto is(archive.istream(path));
    auto tex(cv::imdecode(is->read(), CV_LOAD_IMAGE_COLOR));

    if (!tex.data) {
        LOGTHROW(err2, std::runtime_error)
            << "Unable to load texture from " << is->path() << ".";
    }

    return tex;
}

math::Extents2 computeExtents(const vts::Mesh &mesh)
{
    math::Extents2 extents(math::InvalidExtents{});
    for (const auto &sm : mesh) {
        for (const auto &p : sm.vertices) {
            update(extents, p(0), p(1));
        }
    }
    return extents;
}

vts::TileRange computeTileRange(const math::Extents2 &worldExtents
                                , vts::Lod lod
                                , const math::Extents2 &meshExtents)
{
    vts::TileRange r(math::InvalidExtents{});
    const auto ts(vts::tileSize(worldExtents, lod));
    const auto origin(math::ul(worldExtents));

    for (const auto &p : vertices(meshExtents)) {
        update(r, vts::TileRange::point_type
               ((p(0) - origin(0)) / ts.width
                , (origin(1) - p(1)) / ts.height));
    }

    return r;
}

void Cutter::windowCut(const WindowRecord &wr)
{
    const auto &window(*wr.window);
    const auto &wm(window.mesh);
    LOG(info2) << "Cutting window mesh from " << wm.path << ".";
    auto mesh(vts::loadMeshFromObj(*archive_.meshIStream(wm), wm.path));
    warpInPlace(mesh, vef2world_);

    if (mesh.submeshes.size() != window.atlas.size()) {
        LOGTHROW(err2, std::runtime_error)
            << "Texture/submesh count mismatch in window "
            << window.path << ".";
    }

    vts::opencv::Atlas atlas;
    for (const auto &texture : window.atlas) {
        LOG(info3) << "Loading window texture from: " << texture.path;
        atlas.add(loadTexture(texture.path));
    }

    auto tr(computeTileRange(worldExtents_, wr.lod, computeExtents(mesh)));
    LOG(info3) << "Splitting window " << window.path
               << " to tiles in " << wr.lod << "/" << tr << ".";
    splitToTiles(wr.lod, tr, mesh, atlas);
}

void Cutter::splitToTiles(vts::Lod lod, const vts::TileRange &tr
                          , const vts::Mesh &mesh
                          , const vts::opencv::Atlas &atlas)
{
    for (auto j(tr.ll(1)), je(tr.ur(1)); j <= je; ++j) {
        for (auto i(tr.ll(0)), ie(tr.ur(0)); i <= ie; ++i) {
            tileCut(vts::TileId(lod, i, j), mesh, atlas);
        }
    }
}

math::Extents2 tileExtents(const math::Extents2 &rootExtents
                           , const vts::TileId &tileId)
{
    auto tc(vts::tileCount(tileId.lod));
    auto rs(size(rootExtents));
    math::Size2f ts(rs.width / tc, rs.height / tc);

    return math::Extents2
        (rootExtents.ll(0) + tileId.x * ts.width
         , rootExtents.ur(1) - (tileId.y + 1) * ts.height
         , rootExtents.ll(0) + (tileId.x + 1) * ts.width
         , rootExtents.ur(1) - tileId.y * ts.height);
}

void Cutter::tileCut(const vts::TileId &tileId, const vts::Mesh &mesh
                     , const vts::opencv::Atlas &atlas)
{
    auto extents
        (vts::inflateTileExtents
         (tileExtents(worldExtents_, tileId), clipMargin_));

    vts::Mesh clipped;
    vts::opencv::Atlas clippedAtlas(0); // PNG!

    std::size_t smIndex(0);
    for (const auto &sm : mesh) {
        const auto &texture(atlas.get(smIndex++));

        auto m(vts::clip(sm, extents));
        if (m.empty()) { continue; }

        clipped.submeshes.push_back(std::move(m));
        clippedAtlas.add(texture);
    }

    if (clipped.empty()) { return; }

    // store in temporary storage
    tools::repack(tileId, clipped, clippedAtlas);
    ts_.store(tileId, clipped, clippedAtlas);
}

} // namespace

void cutToTiles(tools::TmpTileset &ts, const vef::Archive &archive
                , const math::Extents2 &worldExtents
                , const geo::CsConvertor &vef2world
                , int maxLod, double clipMargin)
{
    Cutter(ts, archive, worldExtents, vef2world, maxLod, clipMargin)();
}

} // namespace vef
