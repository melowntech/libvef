#ifndef vef2vef_ogrpoly_hpp_included_
#define vef2vef_ogrpoly_hpp_included_

#include <string>
#include <vector>

#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>

#include "math/geometry_core.hpp"

#include "geo/srsdef.hpp"

using Polygon = std::vector<math::Point2_<double>>;
using Polygons = std::vector<Polygon>;

Polygons loadPolygons(const boost::filesystem::path &ogrDataset
                      , const geo::SrsDefinition &srs);

boost::optional<Polygons>
loadPolygons(const boost::optional<boost::filesystem::path> &ogrDataset
             , const geo::SrsDefinition &srs);

#endif // vef2vef_ogrpoly_hpp_included_
