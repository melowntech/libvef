#ifndef vef2vef_ogrpoly_hpp_included_
#define vef2vef_ogrpoly_hpp_included_

#include <string>
#include <vector>

#include <optional>
#include <boost/filesystem/path.hpp>

#include "math/geometry.hpp"

#include "geo/srsdef.hpp"

using Polygon = math::Polygon;
using Polygons = math::MultiPolygon;

Polygons loadPolygons(const boost::filesystem::path &ogrDataset
                      , const geo::SrsDefinition &srs);

std::optional<Polygons>
loadPolygons(const std::optional<boost::filesystem::path> &ogrDataset
             , const geo::SrsDefinition &srs);

Polygons polygonsFromExtents(const math::Extents2 &extents);

std::optional<Polygons>
polygonsFromExtents(const std::optional<math::Extents2> &extents);

#endif // vef2vef_ogrpoly_hpp_included_
