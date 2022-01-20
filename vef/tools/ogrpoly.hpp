#ifndef vef2vef_ogrpoly_hpp_included_
#define vef2vef_ogrpoly_hpp_included_

#include <string>
#include <vector>

#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>

#include "math/geometry.hpp"

#include "geo/srsdef.hpp"

using Polygon = math::Polygon;
using Polygons = math::MultiPolygon;

Polygons loadPolygons(const boost::filesystem::path &ogrDataset
                      , const geo::SrsDefinition &srs);

boost::optional<Polygons>
loadPolygons(const boost::optional<boost::filesystem::path> &ogrDataset
             , const geo::SrsDefinition &srs);

Polygons polygonsFromExtents(const math::Extents2 &extents);

boost::optional<Polygons>
polygonsFromExtents(const boost::optional<math::Extents2> &extents);

#endif // vef2vef_ogrpoly_hpp_included_
