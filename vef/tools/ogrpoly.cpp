#include <algorithm>

#include <gdal_priv.h>
#include <ogrsf_frmts.h>

#include "geo/csconvertor.hpp"

#include "dbglog/dbglog.hpp"

#include "ogrpoly.hpp"

namespace {

using VectorDataset = std::shared_ptr< ::GDALDataset>;

inline double ccw(const math::Point2 &a, const math::Point2 &b
                  , const math::Point2 &c)
{
    return (math::crossProduct(math::Point2(math::normalize(b - a))
                              , math::Point2(math::normalize(c - a)))
            > 0.0);
}

bool ccw(const Polygon &p)
{
    // whatever
    if (p.size() < 3) { return true; }
    if (p.size() == 3) {
        // triangle
        return ccw(p[0], p[1], p[2]);
    }

    // find start point with lowest X-cooridnate; if there are more points with
    // the same lowerst X-coordinate then find the one with lowest Y-coordinate

    std::size_t a(0), b(0), c(0);
    auto start(p.front());

    for (std::size_t j(1), ej(p.size()); j != ej; ++j) {
        const auto &v(p[j]);
        if ((v(0) < start(0))
            || ((v(0) == start(0)) && (v(1) < start(1))))
        {
            start = v;
            b = j;
        }
    }

    const auto last(p.size() - 1);

    if (b == 0) {
        a = last;
        c = 1;
    } else if (b == last) {
        if (p.front() == p.back()) {
            // closed polygon
            a = b - 1;
            c = 1;
        } else {
            a = b - 1;
            c = 0;
        }
    } else {
        a = b - 1;
        c = b + 1;
    }

    return ccw(p[a], p[b], p[c]);
}

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

void reverse(Polygon &polygon)
{
    // reverse all polygons
    Polygon o;
    o.resize(polygon.size());
    std::copy(polygon.rbegin(), polygon.rend(), o.begin());
    polygon.swap(o);
}

void makeCcw(Polygon &polygon)
{
    if (!ccw(polygon)) {
        reverse(polygon);
    }
}

void makeCw(Polygon &polygon)
{
    if (ccw(polygon)) {
        reverse(polygon);
    }
}

Polygons asPolygons(const ::OGRLineString &g)
{
    Polygons polygons;
    polygons.emplace_back();
    auto &polygon(polygons.back());

    for (const auto &p : g) {
        polygon.emplace_back(p.getX(), p.getY());
    }

    makeCcw(polygon);

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
    Polygons polygons;

    for (const auto &ring : g) {
        const auto add(asPolygons(*ring));
        polygons.insert(polygons.end(), add.begin(), add.end());
    }

    if (!polygons.empty()) {
        // make sure first ring is ccw, rest if cw
        auto ipolygons(polygons.begin());
        makeCcw(*ipolygons++);
        for (auto epolygons(polygons.end()); ipolygons != epolygons;
             ++ipolygons)
        {
            makeCw(*ipolygons);
        }
    }

    return polygons;
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

Polygons polygonsFromExtents(const math::Extents2 &extents)
{
    Polygons polygons;

    polygons.emplace_back();
    auto &polygon(polygons.back());

    polygon.push_back(ll(extents));
    polygon.push_back(lr(extents));
    polygon.push_back(ur(extents));
    polygon.push_back(ul(extents));
    polygon.push_back(ll(extents));

    return polygons;
}

boost::optional<Polygons>
polygonsFromExtents(const boost::optional<math::Extents2> &extents)
{
    if (extents) { return polygonsFromExtents(*extents); }
    return boost::none;
}
