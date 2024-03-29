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

#ifndef vef_tiling_hpp_included_
#define vef_tiling_hpp_included_

/** Tiling support. VEF is not tiled by itself but many formats use employ some
 *  kind of tiling hierarchy. It makes sense to put tiling support along VEF.
 */

#include <optional>

#include "reader.hpp"

namespace vef {

using ArchiveCRef = std::reference_wrapper<const Archive>;
using Archives = std::vector<ArchiveCRef>;


struct World {
    geo::SrsDefinition srs;
    math::Extents3 extents;

    World() = default;

    World(const geo::SrsDefinition &srs, const math::Extents3 &extents)
        : srs(srs), extents(extents)
    {}

    World(const geo::SrsDefinition &srs, const math::Extents2 &extents);

    std::ostream& dump(std::ostream &os,
                       const std::string &prefix = {}) const;
};

/** Computed tiling.
 */
struct Tiling {
    math::Extents3 workExtents;
    geo::SrsDefinition workSrs;

    /** Resolution used for tile splitting.
     */
    double resolution;
    int maxLod;

    Tiling(const Archive &archive
           , const math::Size2 &optimalTextureSize
           , bool for3dCutting = false
           , const std::optional<double> &resolution = std::nullopt
           , const std::optional<World> &world = std::nullopt
           , std::size_t extraLods = 0);

    Tiling(const Archive::list &archives
           , const math::Size2 &optimalTextureSize
           , bool for3dCutting = false
           , const std::optional<double> &resolution = std::nullopt
           , const std::optional<World> &world = std::nullopt
           , std::size_t extraLods = 0);

    Tiling(const Archives &archives
           , const math::Size2 &optimalTextureSize
           , bool for3dCutting = false
           , const std::optional<double> &resolution = std::nullopt
           , const std::optional<World> &world = std::nullopt
           , std::size_t extraLods = 0);

    Tiling(std::ostream &os);

    void save(std::ostream &os) const;
};

// inlines

inline Tiling::Tiling(const Archive &archive
                      , const math::Size2 &optimalTextureSize
                      , bool for3dCutting
                      , const std::optional<double> &resolution
                      , const std::optional<World> &world
                      , std::size_t extraLods)
    : Tiling(Archives({std::cref(archive)})
             , optimalTextureSize, for3dCutting, resolution, world
             , extraLods)
{}

inline Tiling::Tiling(const Archive::list &archives
                      , const math::Size2 &optimalTextureSize
                      , bool for3dCutting
                      , const std::optional<double> &resolution
                      , const std::optional<World> &world
                      , std::size_t extraLods)
    : Tiling(Archives(archives.begin(), archives.end())
             , optimalTextureSize, for3dCutting, resolution, world
             , extraLods)
{}

} // namespace vef

#endif // vef_vef_tiling_included_
