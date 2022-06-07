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

#include "dbglog/dbglog.hpp"

#include "utility/scopedguard.hpp"

#include "geo/geodataset.hpp"
#include "geo/gdal.hpp"

#include "imgproc/scanconversion.hpp"

#include "vts-libs/vts/mesh.hpp"

#include "25d.hpp"

namespace fs = boost::filesystem;
namespace vts = vtslibs::vts;

namespace vef {

namespace {

Id selectLod(const LoddedWindow &lw, int sourceLod)
{
    if (sourceLod >= int(lw.lods.size())) {
        LOGTHROW(err2, std::runtime_error)
            << lw.path << ": Invalid source LOD " << sourceLod << ".";
    }

    if (sourceLod < 0) {
        if (-sourceLod >= int(lw.lods.size())) {
            LOGTHROW(err2, std::runtime_error)
                << lw.path << ": Invalid source LOD " << sourceLod << ".";
        }
        return lw.lods.size() + sourceLod;
    }

    return sourceLod;
}

math::Extents2 alignExtents(const math::Extents2 &extents
                            , const math::Point2 &reference
                            , double gsd)
{
    auto sa(reference);

    // pixel reg, sa should be center of the pixel
    sa -= math::Point2(gsd, gsd) / 2;

    math::Extents2 res;
    res.ll = extents.ll - sa;
    res.ur = extents.ur - sa;

    for (int i = 0; i < 2; ++i) {
        res.ll(i) = std::floor(res.ll(i) / gsd) * gsd + sa(i);
        // make the overlap
        res.ur(i) = (std::floor(res.ur(i) / gsd) + 1) * gsd + sa(i);
    }

    return res;
}

/** Geo coordinates to grid mapping.
 * NB: result is in pixel system (pixel centers have integral indices)
 */
inline math::Matrix4 geo2grid(const math::Extents2 &extents
                              , const math::Size2 &gridSize)
{
    math::Matrix4 trafo(boost::numeric::ublas::identity_matrix<double>(4));

    auto es(size(extents));

    // scales
    math::Size2f scale(gridSize.width / es.width
                       , gridSize.height / es.height);

    // scale to grid
    trafo(0, 0) = scale.width;
    trafo(1, 1) = -scale.height;

    // move to origin and also move pixel centers to integral indices
    trafo(0, 3) = -extents.ll(0) * scale.width - 0.5;
    trafo(1, 3) = extents.ur(1) * scale.height - 0.5;

    return trafo;
}

void rasterize(cv::Mat_<double> dem, geo::GeoDataset::Mask &demMask
               , const vts::Mesh &mesh)
{
    // clear mask
    demMask.reset(false);

    std::vector<imgproc::Scanline> scanlines;

    for (const auto &sm : mesh.submeshes) {
        for (const auto &face : sm.faces) {
            const math::Point3 *tri[3] = {
                &sm.vertices[face[0]]
                , &sm.vertices[face[1]]
                , &sm.vertices[face[2]]
            };

            imgproc::scanConvertTriangle
                (*tri[0], *tri[1], *tri[2], 0, dem.rows, scanlines);

            for (const auto &sl : scanlines) {
                imgproc::processScanline(sl, 0, dem.cols
                                         , [&](int x, int y, double z)
                {
                    auto &old(dem(y, x));

                    bool write(!demMask.get(x, y) || (z > old));
                    if (write) {
                        old = z;
                        demMask.set(x, y);
                    }
                });
            }
        }
    }
}

struct Dataset {
    geo::GeoDataset ophoto;
    geo::GeoDataset dem;

    Dataset(geo::GeoDataset &&ophoto, geo::GeoDataset &&dem)
        : ophoto(std::move(ophoto)), dem(std::move(dem))
    {}
};

Dataset rasterize(const Archive &archive
                  , const geo::SrsDefinition &srs
                  , const OptionalMatrix &wtrafo
                  , const math::Extents2 &extents, const double res
                  , const Window &window, const std::string &name
                  , const fs::path &ophotoDir, const fs::path &demDir)
{
    const auto esize(math::size(extents));
    const math::Size2 size(int(std::round(esize.width / res))
                           , int(std::round(esize.height / res)));

    const auto ophotoPath(ophotoDir / (name + ".ophoto.tif"));
    const auto demPath(demDir / (name + ".dem.tif"));

    geo::Gdal::setOption("GDAL_TIFF_INTERNAL_MASK", "YES");
    auto ophoto
        (geo::GeoDataset::create
         (ophotoPath, srs, extents, size
          , geo::GeoDataset::Format::gtiffRGBPhoto()
          , boost::none
          , (geo::GeoDataset::Options
             ("COMPRESS", "JPEG")
             ("JPEG_QUALITY", 98)
             ("TILED", true)
             )
          ));
    auto dem
        (geo::GeoDataset::create
         (demPath, srs, extents, size
          , geo::GeoDataset::Format::dsm()
          , geo::NodataValue(-1e6)
          , (geo::GeoDataset::Options
             ("COMPRESS", "DEFLATE")
             ("PREDICTOR", 3)
             ("ZLEVEL", 9)
             ("TILED", true)
             )
          ));

    // load mesh and convert to grid-local coordinates
    auto mesh(vts::loadMeshFromObj(*archive.meshIStream(window.mesh)
                                   , window.mesh.path));

    {
        auto trafo(geo2grid(extents, size));
        if (wtrafo) {
            trafo = ublas::prod(trafo, *wtrafo);
        }

        for (auto &sm : mesh.submeshes) {
            for (auto &v : sm.vertices) {
                v = transform(trafo, v);
            }
        }
    }

    rasterize(dem.data(), dem.mask(), mesh);

    ophoto.flush();
    dem.flush();

    return Dataset(std::move(ophoto), std::move(dem));
    (void) window;
}

} // namespace

/** Generates lodCount extra LODs from archive[sourceLod] as 2.5 D version of
 *  source meshes.
 */
void generate25d(const fs::path &path, const Archive &archive
                 , int sourceLod, int lodCount
                 , double baseResolution)
{
    const auto srs(archive.manifest().srs.value());

    vef::ArchiveWriter writer(path, true);

    // temporary dir, with cleanup
    const auto tmp(path / "tmp");
    fs::create_directories(tmp);
    //    utility::ScopedGuard tmpCleanup([&tmp]() { fs::remove_all(tmp); });

    const auto ophotoDir(tmp / "ophoto");
    const auto demDir(tmp / "dem");
    fs::create_directories(ophotoDir);
    fs::create_directories(demDir);

    const auto &manifest(archive.manifest());

    std::vector<Dataset> ds;

    for (const auto &lw : manifest.windows) {
        const Id useLod(selectLod(lw, sourceLod));
        double res(baseResolution * (1 << useLod));
        const auto wtrafo(windowMatrix(archive.manifest(), lw));

        const auto &win(lw.lods[useLod]);
        ds.push_back(rasterize(archive, srs, wtrafo
                               , alignExtents(math::extents2(lw.extents)
                                              , {}, res), res
                               , win, lw.name.value(), ophotoDir, demDir));
    }

    writer.flush();

    (void) lodCount;
}

} // namespace vef
