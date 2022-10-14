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
#include <opencv2/highgui/highgui.hpp>

#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/optional/optional_io.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/path.hpp"

#include "math/geometry.hpp"
#include "math/transform.hpp"

#include "geometry/meshop.hpp"

#include "geo/geodataset.hpp"
#include "geo/gdal.hpp"
#include "geo/coordinates.hpp"

#include "imgproc/scanconversion.hpp"
#include "imgproc/binterpolate.hpp"
#include "imgproc/inpaint.hpp"

#include "vts-libs/vts/mesh.hpp"
#include "vts-libs/vts/math.hpp"

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

/** Computes area of 3D mesh and area of its projecteion to X-Y plane.
 *  Projection is determined as an area of triangle with Z cooridnated set to
 *  zero. Therefore, mesh cannot have overlapping trinagles.
 */
std::pair<double, double> meshArea(const geometry::Mesh &mesh)
{
    std::pair<double, double> res(0.0, 0.0);
    auto &area(res.first);
    auto &projected(res.second);

    for (const auto &face : mesh.faces) {
        auto a(mesh.vertices[face.a]);
        auto b(mesh.vertices[face.b]);
        auto c(mesh.vertices[face.c]);

        // area in 3D
        area += vts::triangleArea(a, b, c);

        // reset z coordinate -> area in 2D
        a[2] = b[2] = c[2] = 0.0;
        projected += vts::triangleArea(a, b, c);
    }

    return res;
}

class TileFacesCalculator {
public:
    TileFacesCalculator()
        : base_(1000), roughnessFactorMin_(0.0), roughnessFactorMax_(3.0)
        , quotient_(1.0 - (1.0 / roughnessFactorMax_))
    {}

    int operator()(double meshArea, double meshProjectedArea) const;

private:
    int base_;
    double roughnessFactorMin_;
    double roughnessFactorMax_;
    double quotient_;
};

int TileFacesCalculator::operator()(double meshArea, double meshProjectedArea)
    const
{
    // ratio between mesh area and area of its projection to tile base
    const auto areaRatio(meshArea / meshProjectedArea);

    // calculate scaling factor
    auto factor((std::pow(quotient_, areaRatio) - 1.0)
                      / (quotient_ - 1.0));

    // clamp factor to min/max range
    factor = math::clamp(factor, roughnessFactorMin_, roughnessFactorMax_);

    // apply factor to base number of faces
    return base_* factor;
}

void simplifyMesh(geometry::Mesh &mesh
                  , const TileFacesCalculator &tileFacesCalculator)
{
    // calculate number of faces
    const auto area(meshArea(mesh));
    const int faceCount(tileFacesCalculator(area.first, area.second));
    LOG(info1)
        << "Simplifying mesh to " << faceCount << " faces per tile (from "
        << mesh.faces.size() << ".";

    // simplify with locked inner border

    if (int(faceCount) < int(mesh.faces.size())) {
        // max edge is radius of tile divided by edges per side computed from
        // faces-per-tile
        const auto maxEdgeLength
            (std::sqrt((2 * area.second) / (faceCount / 2.0)));

        mesh = *simplify(mesh, faceCount
                         , geometry::SimplifyOptions
                         (geometry::SimplifyOption::INNERBORDER
                          | geometry::SimplifyOption::CORNERS
                          | geometry::SimplifyOption::PREVENTFACEFLIP)
                         .minAspectRatio(5)
                         .maxEdgeLength(maxEdgeLength)
                         );

        LOG(info1)
            << "Simplified mesh to " << mesh.faces.size()
            << " faces (should be " << faceCount
            << ", difference: " << (int(mesh.faces.size()) - int(faceCount))
            << ").";
    } else {
        LOG(info1) << "No need to simplify mesh.";
    }
}

Id selectLod(const LoddedWindow &lw, int sourceLod)
{
    const int totalLods(lw.lods.size());
    if (sourceLod >= totalLods) {
        LOGTHROW(err2, std::runtime_error)
            << lw.path << ": Invalid source LOD " << sourceLod << ".";
    }

    if (sourceLod < 0) {
        if (-sourceLod >= totalLods) {
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
    // make sure the computed size is at least 1x1
    return math::Size2(std::max(1, int(std::round(esize.width / res)))
                       , std::max(1, int(std::round(esize.height / res))));
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
    LOG(info2) << "Using LOD " << useLod << " (from source LOD "
               << sourceLod << ")";
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
                 , double baseResolution
                 , const boost::optional<int> &generateFromLod)
{
    using namespace std::literals;

    const auto &manifest(archive.manifest());
    const auto &srs(manifest.srs.value());

    vef::ArchiveWriter writer(path, true);
    writer.setSrs(srs);

    // temporary dir, with cleanup
    const auto tmp(path / "tmp");
    fs::create_directories(tmp);

    auto datasets(generateDatasets(tmp, archive, sourceLod, baseResolution));

    // all windows should have the same number of LODs!
    const Id totalLods(manifest.windows.front().lods.size());

    const Id currentEnd
        (generateFromLod
         ? selectLod(manifest.windows.front(), *generateFromLod)
         : totalLods);
    const Id newEnd(totalLods + lodCount);

    const auto &extents(datasets.ophoto.extents());
    const auto l2g(geo::local2geo(extents));

    LOG(info2) << std::fixed << "Extents: " << extents << ".";

    writer.setTrafo(l2g);

    const auto wid(writer.addWindow(boost::none, boost::none
                                    , "ophoto"s));

    math::Extents3 e3(math::InvalidExtents{});

    for (Id lod(0); lod < newEnd; ++lod) {
        const auto lid(writer.addLod(wid));
        auto &amesh(writer.mesh(wid, lid));

        if (lod < currentEnd) {
            writeMesh(amesh, {});
            continue;
        }

        const double txRes(baseResolution * (1 << lod));
        const double meshRes(txRes * 3.0); // TODO: make configurable

        // generate imagery

        // Warping as 16 bits with nodata value set to 256.
        // When saturated to 8 bits nodata value becomes 0 (black),
        // plus we can extract mask.

        const auto txSize(sizeInPixels(math::size(extents), txRes));
        LOG(info2)
            << std::fixed
            << "Computed ophoto size in pixels: " << txSize
            << ", using resolution " << txRes
            << ", extents: " << extents << ".";

        auto ophoto(geo::GeoDataset::deriveInMemory
                    (datasets.ophoto, srs
                     , txSize
                     , extents
                     , GDT_UInt16, geo::NodataValue(256)));
        datasets.ophoto.warpInto
            (ophoto, geo::GeoDataset::Resampling::average);

        const auto size(ophoto.size());
        Image texture(size.height, size.width);
        ophoto.readDataInto(CV_8U, texture);

        Mask mask(ophoto.fetchMask());
        imgproc::jpegBlockInpaint(texture, mask);

        Texture tx;
        tx.size = size;
        tx = writer.addTexture(wid, lod, tx, Texture::Format::jpg);

        cv::imwrite(tx.path.string(), texture
                    , { cv::IMWRITE_JPEG_QUALITY, 90
                        , cv::IMWRITE_PNG_COMPRESSION, 9 });

        // ophoto.copy(utility::addExtension(tx.path, ".tif"), "GTiff");

        // generate valid mesh

        math::Size2 demSize;
        math::Extents2 demExtents;

        // compute DEM extents with added halfpixel around
        {
            const auto esize(math::size(extents));
            demSize = sizeInPixels(esize, meshRes);

            LOG(info2)
                << std::fixed
                << "Computed DEM size in pixels: " << demSize
                << ", using resolution " << meshRes << ".";

            const math::Point2 realMeshRes(esize.width / demSize.width
                                            , esize.height / demSize.height);

            // add one pixel (-> half pixel at each side)
            demSize.width += 1;
            demSize.height += 1;

            const math::Point2 end((demSize.width * realMeshRes(0)) / 2
                                   , (demSize.height * realMeshRes(1)) / 2);

            const auto center(math::center(extents));
            demExtents.ll = center - end;
            demExtents.ur = center + end;

            LOG(info2)
                << std::fixed
                << "Computed inflated DEM extents: " << demExtents << ".";
        }

        auto dem(geo::GeoDataset::deriveInMemory
                 (datasets.dem, srs
                  , demSize
                  , demExtents
                  , GDT_Float64, geo::NodataValue(-1e6)));

        datasets.dem.warpInto
            (dem, geo::GeoDataset::Resampling::average);

        // dem.copy(utility::addExtension(amesh.path, ".tif"), "GTiff");

        // generate (raw) mesh
        geometry::Mesh rmesh;
        dem.exportMesh(rmesh);

        // simplify it
        simplifyMesh(rmesh, TileFacesCalculator());

        // hack: do not take mask into account
        ophoto.mask().reset(true);
        ophoto.flush();

        // texture raw mesh
        geometry::Mesh tmesh;
        ophoto.textureMesh(rmesh, demExtents, tmesh);

        writeMesh(amesh, tmesh);

        math::update(e3, math::computeExtents(tmesh.vertices));
    }

    // set extents in world coords
    writer.setExtents(wid, transform(l2g, e3));

    // done
    writer.flush();
}

} // namespace vef
