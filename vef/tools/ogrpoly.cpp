#include <gdal_priv.h>
#include <ogrsf_frmts.h>

#include "geo/csconvertor.hpp"

#include "dbglog/dbglog.hpp"

#include "ogrpoly.hpp"

namespace {

using VectorDataset = std::shared_ptr< ::GDALDataset>;

VectorDataset openVectorDataset(const std::string &dataset)
{
    auto ds(::GDALOpenEx(dataset.c_str(), (GDAL_OF_VECTOR | GDAL_OF_READONLY)
                         , nullptr, nullptr, nullptr));

    if (!ds) {
        const auto code(::CPLGetLastErrorNo());
        if (code == CPLE_OpenFailed) {
            LOGTHROW(err2, std::runtime_error)
                << "No file found for " << dataset << ".";
        }

        LOGTHROW(err2, std::runtime_error)
            << "Failed to open dataset " << dataset << " ("
            << ::CPLGetLastErrorNo() << ").";
    }

    return VectorDataset(static_cast< ::GDALDataset*>(ds)
                         , [](::GDALDataset *ds) { delete ds; });
}

Polygons asPolygons(const ::OGRLineString &g)
{
    Polygons polygons;
    polygons.emplace_back();
    auto &polygon(polygons.back());

    for (const auto &p : g) {
        polygon.emplace_back(p.getX(), p.getY());
    }

    return polygons;
}

Polygons asPolygons(const ::OGRLinearRing &g)
{
    Polygons polygons;
    polygons.emplace_back();
    auto &polygon(polygons.back());

    for (const auto &p : g) {
        polygon.emplace_back(p.getX(), p.getY());
    }

    return polygons;
}

Polygons asPolygons(const ::OGRPolygon &g)
{
    // ignore holes
    return asPolygons(**g.begin());
}

Polygons asPolygons(const ::OGRMultiPolygon &g)
{
    Polygons polygons;

    for (const auto *p : g) {
        auto add(asPolygons(*p));
        polygons.insert(polygons.end(), add.begin(), add.end());
    }

    return polygons;
}

Polygons asPolygons(const ::OGRGeometry &g)
{
    switch (g.getGeometryType()) {
    case OGRwkbGeometryType::wkbLineString:
        return asPolygons(*g.toLineString());

    case OGRwkbGeometryType::wkbPolygon:
        return asPolygons(*g.toPolygon());

    case OGRwkbGeometryType::wkbMultiPolygon:
        return asPolygons(*g.toMultiPolygon());

    default: break;
    }

    LOGTHROW(err2, std::runtime_error)
        << "Mesh cutting supports only line strings and (multi)polygons.";
    throw; // never reached
}

void convert(Polygons &polygons, const geo::CsConvertor &conv)
{
    for (auto &polygon : polygons) {
        for (auto &p : polygon) {
            p = conv(p);
        }
    }
}

} // namespace

Polygons loadPolygons(const boost::filesystem::path &ogrDataset
                      , const geo::SrsDefinition &srs)
{
    auto ds(openVectorDataset(ogrDataset.string()));

    if (!ds->GetLayerCount()) {
        LOGTHROW(err2, std::runtime_error)
            << "There are no layers in the vector dataset at "
            << ogrDataset << ".";
    }

    geo::CsConvertor conv;

    auto layer(ds->GetLayer(0));
    if (auto ref = layer->GetSpatialRef()) {
        conv = geo::CsConvertor(*ref, srs);
    } else {
        LOGTHROW(err2, std::runtime_error)
            << "There is no SRS assigned to layer <" << layer->GetName()
            << "> in the vector dataset at  " << ogrDataset << ".";
    }

    Polygons polygons;

    for (const ::OGRFeature *f; (f = layer->GetNextFeature()); ) {
        const auto geometry(f->GetGeometryRef());
        auto add(asPolygons(*geometry));
        polygons.insert(polygons.end(), add.begin(), add.end());
    }

    convert(polygons, conv);

    return polygons;
}

boost::optional<Polygons>
loadPolygons(const boost::optional<boost::filesystem::path> &ogrDataset
             , const geo::SrsDefinition &srs)
{
    if (ogrDataset) { return loadPolygons(*ogrDataset, srs); }
    return boost::none;
}
