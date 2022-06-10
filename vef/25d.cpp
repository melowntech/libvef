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

#include <opencv2/imgproc/imgproc.hpp>

#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/optional/optional_io.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/scopedguard.hpp"

#include "geometry/meshop.hpp"

#include "geo/geodataset.hpp"
#include "geo/gdal.hpp"

#include "imgproc/scanconversion.hpp"
#include "imgproc/binterpolate.hpp"

#include "vts-libs/vts/mesh.hpp"

#include "25d.hpp"
#include "utils.hpp"

namespace fs = boost::filesystem;
namespace bio = boost::iostreams;
namespace vts = vtslibs::vts;

namespace vef {

namespace {

typedef cv::Vec3b Pixel;
using Image = cv::Mat_<Pixel>;
using Dem = cv::Mat_<float>;
using Mask = cv::Mat_<std::uint8_t>;

using Matrix2x3 = ublas::matrix<double, ublas::row_major
                                , ublas::bounded_array<double, 6> >;

inline math::Point2 transform(const Matrix2x3 &tr, double x, double y)
{
    return math::Point2
        ((tr(0, 0) * x) + (tr(0, 1) * y) + tr(0, 2)
         , (tr(1, 0) * x) + (tr(1, 1) * y) + tr(1, 2));
}

inline math::Point2 transform(const Matrix2x3 &tr, const math::Point2 &p)
{
    return transform(tr, p(0), p(1));
}

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
inline Matrix2x3 tc2texture(const cv::Mat &tx)
{
    Matrix2x3 trafo(ublas::zero_matrix<double>(2, 3));

    // scale
    trafo(0, 0) = tx.cols;
    trafo(1, 1) = -tx.rows; // flipped axix

    // shift
    trafo(0, 2) = -0.5;
    trafo(1, 2) = tx.rows - 0.5;

    return trafo;
}

Matrix2x3 mesh2texture(const math::Point3 *(&tri)[3]
                       , const math::Point2 (&tc)[3])
{
    // create point mapping
    const cv::Point2f src[3] = {
        cv::Point2f(tri[0]->operator()(0), tri[0]->operator()(1))
        , cv::Point2f(tri[1]->operator()(0), tri[1]->operator()(1))
        , cv::Point2f(tri[2]->operator()(0), tri[2]->operator()(1))
    };

    const cv::Point2f dst[3] = {
        cv::Point2f(tc[0](0), tc[0](1))
        , cv::Point2f(tc[1](0), tc[1](1))
        , cv::Point2f(tc[2](0), tc[2](1))
    };

    // build affine transformation between source and destination matrices
    const cv::Mat_<float> mat(cv::getAffineTransform(src, dst));

    // create trafo
    Matrix2x3 trafo(ublas::zero_matrix<double>(2, 3));
    trafo(0, 0) = mat(0, 0);
    trafo(0, 1) = mat(0, 1);
    trafo(0, 2) = mat(0, 2);
    trafo(1, 0) = mat(1, 0);
    trafo(1, 1) = mat(1, 1);
    trafo(1, 2) = mat(1, 2);
    return trafo;
}

void rasterize(const vts::Mesh &mesh, const Atlas &atlas
               , Dem &dem, Image &ophoto, Mask &mask)
{
    std::size_t smi(0);

    std::vector<imgproc::Scanline> scanlines;
    for (const auto &sm : mesh.submeshes) {
        // load texture
        const auto tx(atlas.get(smi++));
        // make trafo from normalized tc space to image space
        const auto tc2tx(tc2texture(tx));

        auto ifacesTc(sm.facesTc.begin());
        for (const auto &face : sm.faces) {
            const auto facesTc(*ifacesTc++);

            const math::Point3 *tri[3] = {
                &sm.vertices[face[0]]
                , &sm.vertices[face[1]]
                , &sm.vertices[face[2]]
            };

            // convert sm.tc[face[x]] -> image space
            const math::Point2 tc[3] = {
                transform(tc2tx, sm.tc[facesTc[0]])
                , transform(tc2tx, sm.tc[facesTc[1]])
                , transform(tc2tx, sm.tc[facesTc[2]])
            };

            // make mapping between mesh vertex and tx
            const auto mesh2tx(mesh2texture(tri, tc));

            imgproc::scanConvertTriangle
                (*tri[0], *tri[1], *tri[2], 0, dem.rows, scanlines);

            for (const auto &sl : scanlines) {
                imgproc::processScanline(sl, 0, dem.cols
                                         , [&](int x, int y, double z)
                {
                    auto &demValue(dem(y, x));
                    auto &maskValue(mask(y, x));

                    if (!maskValue || (z > demValue)) {
                        demValue = z;

                        // convert (x, y) to texture space
                        auto p(transform(mesh2tx, x, y));
                        ophoto(y, x) = imgproc::rgbInterpolate<Pixel>
                            (tx, p(0), p(1));

                        maskValue = 0xff;
                    }
                });
            }
        }
    }
}

struct Datasets {
    using list = std::vector<Datasets>;

    geo::GeoDataset ophoto;
    geo::GeoDataset dem;

    Datasets(geo::GeoDataset &&ophoto, geo::GeoDataset &&dem)
        : ophoto(std::move(ophoto)), dem(std::move(dem))
    {}

    Datasets()
        : ophoto(geo::GeoDataset::placeholder())
        , dem(geo::GeoDataset::placeholder())
    {}
};


math::Size2 sizeInPixels(const math::Size2f &esize, double res)
{
    return math::Size2(int(std::round(esize.width / res))
                       , int(std::round(esize.height / res)));
}

Datasets createDatasets(const geo::SrsDefinition &srs
                        , const math::Extents2 &extents, const double res
                        , const fs::path &dir)
{
    const auto ophotoPath(dir / "ophoto.tif");
    const auto demPath(dir / "dem.tif");

    const auto size(sizeInPixels(math::size(extents), res));

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
          , boost::none
          , (geo::GeoDataset::Options
             ("COMPRESS", "DEFLATE")
             ("PREDICTOR", 3)
             ("ZLEVEL", 9)
             ("TILED", true)
             )
          ));

    return Datasets(std::move(ophoto), std::move(dem));
}

void rasterize(const Archive &archive, const math::Matrix4 &trafo
               , const Window &window, const std::string &name
               , Dem &demMat, Image &ophotoMat, Mask &mask)
{
    LOG(info3) << "Rasterizing window <" << name << ">.";

    // load atlas and mesh
    auto mesh(vts::loadMeshFromObj(*archive.meshIStream(window.mesh)
                                   , window.mesh.path));
    const auto atlas(loadAtlas(archive, window));

    if (atlas.size() != mesh.submeshes.size()) {
        LOGTHROW(err2, std::runtime_error)
            << "Atlas and mesh count does not match.";
    }

    // convert mesh to grid-local cooridnates
    {
        for (auto &sm : mesh.submeshes) {
            for (auto &v : sm.vertices) {
                v = transform(trafo, v);
            }
        }
    }

    rasterize(mesh, atlas, demMat, ophotoMat, mask);
}

Datasets generateDatasets(const fs::path &path, const Archive &archive
                          , int sourceLod, double baseResolution)
{
    const auto srs(archive.manifest().srs.value());
    const auto &manifest(archive.manifest());

    // accumulate extents and compute source LOD
    // NB: all windows should have the same LOD depth!
    Id useLod(selectLod(manifest.windows.front(), sourceLod));
    double res(baseResolution * (1 << useLod));

    math::Extents2 extents(math::InvalidExtents{});
    for (const auto &lw : manifest.windows) {
        math::update(extents, math::extents2(lw.extents));
    }
    extents = alignExtents(extents, {}, res);

    auto datasets(createDatasets(srs, extents, res, path));
    auto size(datasets.dem.size());

    auto trafo(geo2grid(extents, size));

    Dem demMat(size.height, size.width, -1e6f);
    Image ophotoMat(size.height, size.width, Pixel());
    Mask mask(size.height, size.width, std::uint8_t(0));

    for (const auto &lw : manifest.windows) {
        const auto wtrafo(windowMatrix(archive.manifest(), lw));
        const auto &win(lw.lods[useLod]);

        math::Matrix4 t(wtrafo
                        ? ublas::prod(trafo, *wtrafo)
                        : trafo);

        rasterize(archive, t, win, lw.name.value()
                  , demMat, ophotoMat, mask);
    }

    datasets.dem.writeBlock({}, demMat);
    datasets.dem.writeMaskBlock({}, mask);
    datasets.ophoto.writeBlock({}, ophotoMat);
    datasets.ophoto.writeMaskBlock({}, mask);

    datasets.dem.flush();
    datasets.ophoto.flush();

    datasets.dem.buildOverviews
        (geo::GeoDataset::Resampling::average
         , datasets.dem.binaryDecimation({2, 2}));
    datasets.ophoto.buildOverviews
        (geo::GeoDataset::Resampling::average
         , datasets.ophoto.binaryDecimation({2, 2}));

    return datasets;
}

void writeMesh(const Mesh &amesh, const geometry::Mesh &mesh)
{
    // TODO: use proper setup
    geometry::ObjStreamSetup setup;

    const auto mtlName(amesh.mtlPath().filename().string());
    switch (amesh.format) {
    case vef::Mesh::Format::obj:
        // plain
        saveAsObj(mesh, amesh.path, mtlName, setup);
        break;

    case vef::Mesh::Format::gzippedObj:
        // gzipped
        saveAsGzippedObj(mesh, amesh.path, mtlName, setup);
        break;
    }
}

} // namespace

/** Generates lodCount extra LODs from archive[sourceLod] as 2.5 D version of
 *  source meshes.
 */
void generate25d(const fs::path &path, const Archive &archive
                 , int sourceLod, int lodCount
                 , double baseResolution)
{
    const auto &manifest(archive.manifest());

    vef::ArchiveWriter writer(path, true);
    writer.setSrs(manifest.srs.value());

    // temporary dir, with cleanup
    const auto tmp(path / "tmp");
    fs::create_directories(tmp);
    //    utility::ScopedGuard tmpCleanup([&tmp]() { fs::remove_all(tmp); });

    auto datasets(generateDatasets(tmp, archive, sourceLod, baseResolution));

    // all windows should have the same number of LODs!
    const Id currentEnd(manifest.windows.front().lods.size());
    const Id newEnd(currentEnd + lodCount);

    for (const auto &lw : manifest.windows) {
        const auto wid(writer.addWindow(boost::none, boost::none
                                        , lw.name.value()));

        for (Id lod(0); lod < newEnd; ++lod) {
            const auto lid(writer.addLod(wid));

            if (lod < currentEnd) {
                LOG(info4) << "original LOD: <" << lod << ">";
                writeMesh(writer.mesh(wid, lid), {});
                continue;
            }

            LOG(info4) << "new LOD: <" << lod << ">";
            // TODO: generate valid mesh
            writeMesh(writer.mesh(wid, lid), {});
        }
    }

    writer.flush();

    (void) datasets;
}

} // namespace vef
