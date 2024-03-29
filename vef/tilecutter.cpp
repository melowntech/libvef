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

#include <atomic>

#include <opencv2/highgui/highgui.hpp>

#include "utility/openmp.hpp"
#include "math/transform.hpp"
#include "math/extent.hpp"


#include "vts-libs/vts/meshop.hpp"
#include "vts-libs/tools-support/repackatlas.hpp"

#include "tilecutter.hpp"
#include "utils.hpp"

namespace fs = boost::filesystem;

namespace vts = vtslibs::vts;
namespace vs = vtslibs::storage;
namespace tools = vtslibs::vts::tools;

namespace vef {

namespace {

/** Per-window record to ease parallel processing.
 */
struct WindowRecord {
    const vef::Window *window;
    vts::Lod lod;
    vef::OptionalMatrix trafo;

    WindowRecord(const vef::Window &window, vts::Lod lod
                 , const vef::OptionalMatrix &trafo)
        : window(&window), lod(lod), trafo(trafo)
    {}

    typedef std::vector<WindowRecord> list;
};

WindowRecord::list windowRecordList(const vef::Archive &archive
                                    , const TileCutterConfig &config)
{
    WindowRecord::list list;

    for (const auto &lw : archive.manifest().windows) {
        const auto trafo(vef::windowMatrix(archive.manifest(), lw));
        auto lod(config.maxLod);
        const auto topLod(config.tileExtents
                          ? config.tileExtents->lod
                          : 0);

        std::size_t bLod(0);
        std::size_t eLod(lw.lods.size());

        if (config.lodDepth > 0) {
            // >0 -> only first lodDepth lods
            eLod = std::min(std::size_t(config.lodDepth), eLod);
        } else if (config.lodDepth < 0) {
            // <0 -> only last lodDepth lods
            const std::size_t ld(-config.lodDepth);
            if (ld < eLod) {
                bLod = eLod - ld;
            }
        }

        std::size_t i(0);
        for (const auto &w : lw.lods) {
            if (lod < topLod) {
                // tile extents limit
                break;
            }
            if ((i >= bLod) && (i < eLod)) {
                list.emplace_back(w, lod, trafo);
            }
            --lod;
            ++i;
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

inline void transformInPlace(vts::SubMesh &mesh, const math::Matrix4 &trafo)
{
    for (auto &v : mesh.vertices) { v = math::transform(trafo, v); }
}

inline void transformInPlace(vts::Mesh &mesh, const OptionalMatrix &trafo)
{
    if (!trafo) { return; }
    for (auto &sm : mesh) { transformInPlace(sm, *trafo); }
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

math::Extents2 horizontalExtents(const TileCutterConfig &config)
{
    return std::visit([](const auto &e) { return math::extents2(e); }
        , config.worldExtents);
}

math::Extent verticalExtent(const TileCutterConfig &config)
{
    return std::visit([](const auto &e) { return math::extent(e, 2); }
        , config.worldExtents);
}

vts::Ranges computeClipRanges(const TileCutterConfig &config)
{
    if (config.tileExtents) {
        return vts::Ranges(*config.tileExtents, config.maxLod);
    }
    return {};
}

class Cutter {
public:
    Cutter(tools::TmpTileset &ts, const vef::Archive &archive
           , const TileCutterConfig &config)
        : ts_(ts), archive_(archive), config_(config)
        , worldExtents_(horizontalExtents(config_))
        , verticalExtent_(verticalExtent(config_))
        , clipRanges_(computeClipRanges(config_))
        , windows_(windowRecordList(archive_, config))
        , generated_(), total_(windows_.size())
    {
        LOG(info3) << "Cutting " << archive.path() << " into tiles.";
    }

    void operator()(/**vt::ExternalProgress &progress*/);

private:
    void windowCut(const WindowRecord &window, const geo::CsConvertor &conv);

    void splitToTiles(vts::Lod lod, const vts::TileRange &tr
                      , const vts::Mesh &mesh
                      , const vts::opencv::Atlas &atlas);

    void tileCut(const vts::TileId &tileId, const vts::Mesh &mesh
                 , const vts::opencv::Atlas &atlas);

    geo::CsConvertor vef2world() const;

    tools::TmpTileset &ts_;
    const vef::Archive &archive_;
    const TileCutterConfig config_;
    const math::Extents2 worldExtents_;
    const math::Extent verticalExtent_;
    const vts::Ranges clipRanges_;
    WindowRecord::list windows_;
    std::atomic<std::size_t> generated_;
    std::size_t total_;
};

geo::CsConvertor Cutter::vef2world() const
{
    if (!config_.dstSrs) { return {}; }
    return geo::CsConvertor(archive_.manifest().srs.value()
                            , *config_.dstSrs);
}

void Cutter::operator()(/**vt::ExternalProgress &progress*/)
{
    UTILITY_OMP(parallel)
    {
        const auto convertor(vef2world());
        UTILITY_OMP(for schedule(dynamic, 1))
        for (std::size_t i = 0; i < windows_.size(); ++i) {
            windowCut(windows_[i], convertor);
        }
    }

    ts_.flush();
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

math::Extent computeVerticalExtent(const vts::SubMesh &sm)
{
    math::Extent extent(math::InvalidExtents{});
    for (const auto &p : sm.vertices) { update(extent, p(2)); }
    return extent;
}

vts::TileSpan computeVerticalTileSpan(const math::Extent &rootExtent
                                      , vts::Lod lod
                                      , const math::Extent &meshExtent)
{
    vts::TileSpan s(math::InvalidExtents{});
    const auto ts(vts::tileSize(rootExtent, lod));
    const auto origin(math::l(rootExtent));

    for (const auto &p : { meshExtent.l, meshExtent.r }) {
        update(s, vts::TileSpan::point_type((p - origin) / ts));
    }

    return s;
}

std::optional<vts::TileRange>
clipRange(const vts::Ranges &ranges, vts::Lod lod)
{
    if (const auto range = ranges.tileRange(lod, std::nothrow)) {
        return *range;
    }
    return std::nullopt;
}

vts::TileRange
computeTileRange(const math::Extents2 &worldExtents
                 , vts::Lod lod
                 , const math::Extents2 &meshExtents
                 , const std::optional<vts::TileRange> &clipRange)
{
    vts::TileRange r(math::InvalidExtents{});
    const auto ts(vts::tileSize(worldExtents, lod));
    const auto origin(math::ul(worldExtents));

    for (const auto &p : vertices(meshExtents)) {
        update(r, vts::TileRange::point_type
               ((p(0) - origin(0)) / ts.width
                , (origin(1) - p(1)) / ts.height));
    }

    // apply tile extents at given LOD
    if (clipRange) {
        r = vts::tileRangesIntersect(r, *clipRange, std::nothrow);
    }

    return r;
}

void Cutter::windowCut(const WindowRecord &wr, const geo::CsConvertor &conv)
{
    const auto &window(*wr.window);

    if (config_.tileExtents && (wr.lod < config_.tileExtents->lod)) {
        // FIXME: this should be checked in generation of window records
        Done done(++generated_, total_);
        LOG(info3)
            << "Skipped window outside of configured tile extents "
            << done << ": " << window.path<< ".";
        return;
    }

    const auto &wm(window.mesh);
    LOG(info2) << "Cutting window mesh from " << wm.path << ".";
    auto mesh(vts::loadMeshFromObj(*archive_.meshIStream(wm), wm.path));
    transformInPlace(mesh, wr.trafo);
    warpInPlace(mesh, conv);

    if (mesh.empty()) {
        Done done(++generated_, total_);
        LOG(info3)
            << "Skipped empty window " << done << ": " << window.path << ".";
        return;
    }

    if (mesh.submeshes.size() != window.atlas.size()) {
        LOGTHROW(err2, std::runtime_error)
            << "Texture/submesh count mismatch in window "
            << window.path << ".";
    }

    const auto atlas(loadAtlas(archive_, window));

    auto tr(computeTileRange
            (worldExtents_, wr.lod, computeExtents(mesh)
             , clipRange(clipRanges_, wr.lod)));
    if (!math::valid(tr)) {
        Done done(++generated_, total_);
        LOG(info3)
            << "Skipped window outside of configured tile extents "
            << done << ": " << window.path<< ".";
        return;
    }

    LOG(info2) << "Splitting window " << window.path
               << " to tiles in " << wr.lod << "/" << tr << ".";
    splitToTiles(wr.lod, tr, mesh, atlas);

    Done done(++generated_, total_);

    LOG(info3)
        << "Split window " << done << ": " << window.path
        << " to tiles in " << wr.lod << "/" << tr << ".";
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

math::Extent tileVerticalExtent(const math::Extent &rootExtent
                                , const vts::Lod lod
                                , vts::TileSpan::value_type z)
{
    auto ts(vts::tileSize(rootExtent, lod));
    return math::Extent
        (rootExtent.l + z * ts, rootExtent.l + (z + 1) * ts);
}

/** Inflates tile vertial extent by given margin in both directions.
 *
 * If margin is positive then it is used as a fraction of tile
 * height. Otherwise absolute value of margin is treated as margin itself.
 */
math::Extent inflateTileExtent(const math::Extent &extent, double margin)
{
    const auto m((margin < 0.0)
                 ? -margin // margin itself
                 : margin * math::size(extent)); // fraction

    return math::Extent(extent.l - m, extent.r + m);
}

void Cutter::tileCut(const vts::TileId &tileId, const vts::Mesh &mesh
                     , const vts::opencv::Atlas &atlas)
{
    auto extents
        (vts::inflateTileExtents
         (tileExtents(worldExtents_, tileId), config_.clipMargin));

    vts::Mesh clipped;
    vts::opencv::Atlas clippedAtlas(0); // PNG!

    std::size_t smIndex(0);
    for (const auto &sm : mesh) {
        const auto &texture(atlas.get(smIndex++));

        auto m(vts::clip(sm, extents));
        if (m.empty()) { continue; }

        // cut again if cutting to cubes
        if (math::valid(verticalExtent_)) {
            auto ts(computeVerticalTileSpan(verticalExtent_, tileId.lod
                                            , computeVerticalExtent(sm)));
            for (auto i(ts.l); i <= ts.r; ++i) {
                auto ve(inflateTileExtent
                        (tileVerticalExtent(verticalExtent_, tileId.lod, i)
                         , config_.clipMargin));

                auto vm(vts::clip(m, ve));
                if (vm.empty()) { continue; }

                // store third tile-id component in z-index
                vm.zIndex = i;

                clipped.submeshes.push_back(std::move(vm));
                clippedAtlas.add(texture);
            }
            continue;
        }

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
                , const std::optional<geo::SrsDefinition> &dstSrs
                , int maxLod, double clipMargin
                , int lodDepth)
{
    TileCutterConfig config;
    config.worldExtents = worldExtents;
    config.dstSrs = dstSrs;
    config.maxLod = maxLod;
    config.clipMargin = clipMargin;
    config.lodDepth = lodDepth;

    Cutter(ts, archive, config)();
}

void cutToTiles(tools::TmpTileset &ts, const vef::Archive &archive
                , const math::Extents3 &worldExtents
                , const std::optional<geo::SrsDefinition> &dstSrs
                , int maxLod, double clipMargin
                , int lodDepth)
{
    TileCutterConfig config;
    config.worldExtents = worldExtents;
    config.dstSrs = dstSrs;
    config.maxLod = maxLod;
    config.clipMargin = clipMargin;
    config.lodDepth = lodDepth;

    Cutter(ts, archive, config)();
}

void cutToTiles(vtslibs::vts::tools::TmpTileset &ts
                , const vef::Archive &archive
                , const TileCutterConfig &config)
{
    Cutter(ts, archive, config)();
}

} // namespace vef
