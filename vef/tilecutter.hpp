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

#ifndef vef_tilecutter_hpp_included_
#define vef_tilecutter_hpp_included_

/** Tile cutting. VEF is not tiled by itself but many formats use employ some
 *  kind of tiling hierarchy. It makes sense to put tile cutting support along
 *  VEF.
 */

#include <optional>
#include <variant>

#include "vts-libs/tools-support/tmptileset.hpp"

#include "reader.hpp"

namespace vef {

/** Cut source meshes to tiles in binomic division. World is specified by
 *  `worldExtents` (i.e. tile 0-0-0). Meshes from archive are converted to world
 *  using provided `vef2world` convertor.
 *
 *  Archive's original data is mapped to `maxLod`.
 *
 *  Tiles are clipped with given tile margin.
 *
 *  Output is stored in the provided temporary tileset.
 *
 *  \param lodDepth:
 *      =0: nothing happens
 *      >0: only bottom lodDepth lods are processed from the input
 *      <0: only top -lodDepth lods are processed from the input
 */
void cutToTiles(vtslibs::vts::tools::TmpTileset &ts
                , const vef::Archive &archive
                , const math::Extents2 &worldExtents
                , const std::optional<geo::SrsDefinition> &dstSrs
                , int maxLod, double clipMargin = 1.0 / 128.
                , int lodDepth = 0);

/** Cut source meshes to tiles in binomic division. World is specified by
 *  `worldExtents` (i.e. tile 0-0-0). Meshes from archive are converted to world
 *  using provided `vef2world` convertor.
 *
 *  Archive's original data is mapped to `maxLod`.
 *
 *  Tiles are clipped with given tile margin.
 *
 *  Output is stored in the provided temporary tileset.
 *
 *  \param lodDepth:
 *      =0: nothing happens
 *      >0: only bottom lodDepth lods are processed from the input
 *      <0: only top -lodDepth lods are processed from the input
 */
void cutToTiles(vtslibs::vts::tools::TmpTileset &ts
                , const vef::Archive &archive
                , const math::Extents3 &worldExtents
                , const std::optional<geo::SrsDefinition> &dstSrs
                , int maxLod, double clipMargin = 1.0 / 128.
                , int lodDepth = 0);

using WorldExtents = std::variant<math::Extents2, math::Extents3>;

struct TileCutterConfig {
    /** Extents2 (for quadro division) or Extents3 (for octo division)
     */
    WorldExtents worldExtents;

    /** Destination SRS, if different than input srs.
     */
    std::optional<geo::SrsDefinition> dstSrs;

    /** Maximum lod to process from inpout
     */
    int maxLod = 0;

    /** Clipping margin:
     * >=0: margin = tileSize * clipMargin  # fraction of tile size
     *  <0: margin = -clipMargin            # absolute dimension
     */
    double clipMargin = 1.0 / 128.;

    /** =0: nothing happens
     *  >0: only bottom lodDepth lods are processed from the input
     *  <0: only top -lodDepth lods are processed from the input
     */
    int lodDepth = 0;

    /** Limit output to given tile subtree if set.
     */
    std::optional<vtslibs::vts::LodTileRange> tileExtents;
};

void cutToTiles(vtslibs::vts::tools::TmpTileset &ts
                , const vef::Archive &archive
                , const TileCutterConfig &config);

} // namespace vef

#endif // vef_vef_tilecutter_included_
