#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Area.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/geometry/RegulatoryElement.h>

#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/geometries/register/multi_linestring.hpp>
#include <boost/python/args.hpp>
#include <boost/python/default_call_policies.hpp>
#include <boost/python/make_constructor.hpp>

#include "lanelet2_core/Forward.h"
#include "lanelet2_core/geometry/Point.h"
#include "lanelet2_core/primitives/CompoundLineString.h"
#include "lanelet2_core/primitives/Lanelet.h"
#include "lanelet2_core/primitives/LineString.h"
#include "lanelet2_python/internal/converter.h"

BOOST_GEOMETRY_REGISTER_MULTI_LINESTRING(lanelet::LineStrings2d)
BOOST_GEOMETRY_REGISTER_MULTI_LINESTRING(lanelet::ConstLineStrings2d)
BOOST_GEOMETRY_REGISTER_MULTI_LINESTRING(lanelet::ConstHybridLineStrings2d)
BOOST_GEOMETRY_REGISTER_MULTI_LINESTRING(lanelet::CompoundLineStrings2d)

using namespace boost::python;
using namespace lanelet;

using HybridLs3d = ConstHybridLineString3d;
using HybridLs2d = ConstHybridLineString2d;

template <typename PrimT>
auto wrapFindNearest() {
  using ResultT = std::vector<std::pair<double, PrimT>>;
  using Sig = ResultT (*)(PrimitiveLayer<PrimT>&, const BasicPoint2d&, unsigned);
  auto func = static_cast<Sig>(lanelet::geometry::findNearest);
  converters::PairConverter<std::pair<double, PrimT>>();
  converters::VectorToListConverter<ResultT>();
  return def("findNearest", func, (arg("layer"), arg("point"), arg("n") = 1),
             "Find the n nearest elements to the given point in this layer of a lanelet map. Returns a list of tuples "
             "(distance, primitive)");
}
template <typename PrimT, typename GeometryT>
auto wrapFindWithin2d() {
  using ResultT = std::vector<std::pair<double, PrimT>>;
  using Sig = ResultT (*)(PrimitiveLayer<PrimT>&, const GeometryT&, double);
  auto func = static_cast<Sig>(lanelet::geometry::findWithin2d);
  return def("findWithin2d", func, (arg("layer"), arg("geometry"), arg("maxDist") = 0),
             "Returns all elements in this layer that are closer than maxDist to a geometry in 2D");
}
template <typename PrimT, typename GeometryT>
auto wrapFindWithin3d() {
  using ResultT = std::vector<std::pair<double, PrimT>>;
  using Sig = ResultT (*)(PrimitiveLayer<PrimT>&, const GeometryT&, double);
  auto func = static_cast<Sig>(lanelet::geometry::findWithin3d);
  return def("findWithin3d", func, (arg("layer"), arg("geometry"), arg("maxDist") = 0),
             "Returns all elements in this layer that are closer than maxDist to a geometry in 3D");
}

std::vector<BasicPoint2d> toBasicVector(const BasicPoints2d& pts) {
  return utils::transform(pts, [](auto& p) { return BasicPoint2d(p.x(), p.y()); });
}

template <typename T>
lanelet::BoundingBox2d boundingBox2dFor(const T& t) {
  return lanelet::geometry::boundingBox2d(t);
}

template <typename T>
lanelet::BoundingBox3d boundingBox3dFor(const T& t) {
  return lanelet::geometry::boundingBox3d(t);
}

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define TO2D_AS(X)                        \
  if (extract<X>(o).check()) {            \
    return object(to2D(extract<X>(o)())); \
  }

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define TO3D_AS(X)                        \
  if (extract<X>(o).check()) {            \
    return object(to3D(extract<X>(o)())); \
  }

template <typename PtT>
double distancePointToLss(const PtT& p, const object& lss) {
  auto distance = [](PtT p, auto range) {
    return boost::geometry::distance(p, utils::transform(range, [](auto& v) { return utils::toHybrid(v); }));
  };
  if (extract<ConstLineStrings2d>(lss).check()) {
    return distance(p, extract<ConstLineStrings2d>(lss)());
  }
  return distance(p, extract<LineStrings2d>(lss)());
}

object to2D(object o) {
  using utils::to2D;
  TO2D_AS(Point3d)
  TO2D_AS(BasicPoint3d)
  TO2D_AS(ConstPoint3d)
  TO2D_AS(LineString3d)
  TO2D_AS(LineString3d)
  TO2D_AS(ConstLineString3d)
  TO2D_AS(Polygon3d)
  TO2D_AS(ConstPolygon3d)
  TO2D_AS(CompoundLineString3d)
  return o;
}

object to3D(object o) {
  using utils::to3D;
  TO3D_AS(Point2d)
  TO3D_AS(BasicPoint2d)
  TO3D_AS(ConstPoint2d)
  TO3D_AS(LineString2d)
  TO3D_AS(LineString2d)
  TO3D_AS(ConstLineString2d)
  TO3D_AS(Polygon2d)
  TO3D_AS(ConstPolygon2d)
  TO3D_AS(CompoundLineString2d)
  return o;
}
#undef TO2D_AS
#undef TO3D_AS

template <typename T1, typename T2>
bool equalsLs(const T1& ls1, const T2& ls2) {
  return boost::geometry::equals(ls1.basicLineString(), ls2.basicLineString());
}

BOOST_PYTHON_MODULE(PYTHON_API_MODULE_NAME) {  // NOLINT
  namespace lg = lanelet::geometry;

  def("to2D", to2D, arg("primitive"), "Convert a 3D primitive to its 2D pendant (e.g. a Point3d to a Point2d)");
  def("to3D", to3D, arg("primitive"), "Convert a 2D primitive to its 3D pendant (e.g. a Point2d to a Point3d)");

  // p2p
  const auto distarg = (arg("primitive1"), arg("primitive2"));
  const auto* p2phelp = "2D distance between points";
  def("distance", lg::distance<BasicPoint2d, BasicPoint2d>, distarg, p2phelp);
  def("distance", lg::distance<ConstPoint2d, ConstPoint2d>, distarg, p2phelp);
  def("distance", lg::distance<ConstPoint2d, BasicPoint2d>, distarg, p2phelp);
  def("distance", lg::distance<BasicPoint2d, ConstPoint2d>, distarg, p2phelp);

  // p2l
  const auto* p2lhelp = "Shortest distance between a point and a linestring in 2D";
  const auto* l2phelp = "Shortest distance between a linestring and a point in 2D";
  def("distance", lg::distance<ConstPoint2d, HybridLs2d>, distarg, p2lhelp);
  def("distance", lg::distance<HybridLs2d, ConstPoint2d>, distarg, l2phelp);
  def("distance", lg::distance<ConstPoint2d, CompoundLineString2d>, distarg, p2lhelp);
  def("distance", lg::distance<CompoundLineString2d, ConstPoint2d>, distarg, l2phelp);
  def("distance", lg::distance<ConstPoint2d, ConstLineString2d>, distarg, p2lhelp);
  def("distance", lg::distance<ConstLineString2d, ConstPoint2d>, distarg, l2phelp);
  def("distance", lg::distance<ConstPoint2d, CompoundLineString2d>, distarg, p2lhelp);
  def("distance", lg::distance<CompoundLineString2d, ConstPoint2d>, distarg, l2phelp);
  def("distance", lg::distance<BasicPoint2d, ConstLineString2d>, distarg, p2lhelp);
  def("distance", lg::distance<ConstLineString2d, BasicPoint2d>, distarg, l2phelp);
  def("distance", lg::distance<BasicPoint2d, CompoundLineString2d>, distarg, p2lhelp);
  def("distance", lg::distance<CompoundLineString2d, BasicPoint2d>, distarg, l2phelp);

  // l2l
  const auto* l2lhelp = "Shortest distance between two linestrings in 2D";
  def(
      "distance", +[](const ConstLineString2d& ls1, const ConstLineString2d& ls2) { return lg::distance2d(ls1, ls2); },
      distarg, l2lhelp);
  def("distance", lg::distance<ConstHybridLineString2d, ConstHybridLineString2d>, distarg, l2lhelp);
  def(
      "distance",
      +[](const CompoundLineString2d& ls1, const CompoundLineString2d& ls2) { return lg::distance2d(ls1, ls2); },
      distarg, l2lhelp);
  def(
      "distance",
      +[](const ConstLineString2d& ls1, const CompoundLineString2d& ls2) { return lg::distance2d(ls1, ls2); }, distarg,
      l2lhelp);
  def(
      "distance",
      +[](const CompoundLineString2d& ls1, const ConstLineString2d& ls2) { return lg::distance2d(ls1, ls2); }, distarg,
      l2lhelp);

  // poly2p
  const auto* p2polyhelp = "Shortest distance between a point and a polygon in 2D";
  const auto* poly2phelp = "Shortest distance between a polygon and a point in 2D";
  def("distance", lg::distance<ConstHybridPolygon2d, BasicPoint2d>, distarg, poly2phelp);
  def("distance", lg::distance<BasicPoint2d, ConstHybridPolygon2d>, distarg, p2polyhelp);
  def(
      "distance", +[](const ConstPolygon2d& p1, const BasicPoint2d& p2) { return lg::distance2d(p1, p2); }, distarg,
      poly2phelp);
  def(
      "distance", +[](const BasicPoint2d& p1, const ConstPolygon2d& p2) { return lg::distance2d(p1, p2); }, distarg,
      p2polyhelp);
  def(
      "distance", +[](const ConstPolygon2d& p1, const ConstPoint2d& p2) { return lg::distance2d(p1, p2.basicPoint()); },
      distarg, poly2phelp);
  def(
      "distance", +[](const ConstPoint2d& p1, const ConstPolygon2d& p2) { return lg::distance2d(p1.basicPoint(), p2); },
      distarg, p2polyhelp);
  def(
      "distance",
      +[](const CompoundPolygon2d& p1, const ConstPoint2d& p2) {
        return lg::distance(p1.basicPolygon(), p2.basicPoint());
      },
      distarg, poly2phelp);
  def(
      "distance",
      +[](const ConstPoint2d& p1, const CompoundPolygon2d& p2) {
        return lg::distance(p2.basicPolygon(), p1.basicPoint());
      },
      distarg, p2polyhelp);
  def(
      "distance",
      +[](const CompoundPolygon2d& p1, const BasicPoint2d& p2) { return lg::distance(p1.basicPolygon(), p2); }, distarg,
      poly2phelp);
  def(
      "distance",
      +[](const BasicPoint2d& p1, const CompoundPolygon2d& p2) { return lg::distance(p2.basicPolygon(), p1); }, distarg,
      p2polyhelp);

  // poly2ls
  const auto* poly2lshelp = "Shortest distance in 2D between a polygon and a linestring";
  const auto* ls2polyhelp = "Shortest distance in 2D between a linestring and a polygon";
  def(
      "distance", +[](const ConstPolygon2d& p1, const ConstLineString2d& p2) { return lg::distance2d(p1, p2); },
      distarg, poly2lshelp);
  def("distance", lg::distance2d<ConstHybridPolygon2d, ConstHybridLineString2d>, distarg, poly2lshelp);
  def(
      "distance", +[](const ConstLineString2d& p2, const ConstPolygon2d& p1) { return lg::distance2d(p1, p2); },
      distarg, ls2polyhelp);
  def("distance", lg::distance2d<ConstHybridLineString2d, ConstHybridPolygon2d>, distarg, ls2polyhelp);
  def(
      "distance", +[](const ConstPolygon2d& p1, const CompoundLineString2d& p2) { return lg::distance2d(p1, p2); },
      distarg, poly2lshelp);
  def(
      "distance", +[](const CompoundLineString2d& p1, const CompoundPolygon2d& p2) { return lg::distance2d(p1, p2); },
      distarg, ls2polyhelp);
  def(
      "distance", +[](const ConstLineString2d& p2, const CompoundPolygon2d& p1) { return lg::distance2d(p1, p2); },
      distarg, ls2polyhelp);
  def(
      "distance", +[](const CompoundPolygon2d& p2, const ConstLineString2d& p1) { return lg::distance2d(p1, p2); },
      distarg, poly2lshelp);
  def("distance", lg::distance2d<ConstHybridLineString2d, CompoundPolygon2d>, distarg, ls2polyhelp);
  def(
      "distance", +[](const CompoundPolygon2d& p1, const CompoundLineString2d& p2) { return lg::distance2d(p1, p2); },
      distarg, poly2lshelp);
  def(
      "distance", +[](const CompoundLineString2d& p1, const CompoundPolygon2d& p2) { return lg::distance2d(p1, p2); },
      distarg, ls2polyhelp);

  // poly2poly
  const auto* poly2polyhelp = "Shortest distance in 2D between two polygons";
  def("distance", lg::distance<ConstHybridPolygon2d, ConstHybridPolygon2d>, distarg, poly2polyhelp);
  def(
      "distance", +[](const ConstPolygon2d& p1, const ConstPolygon2d& p2) { return lg::distance2d(p1, p2); }, distarg,
      poly2polyhelp);
  def(
      "distance", +[](const ConstHybridPolygon2d& p1, const ConstPolygon2d& p2) { return lg::distance2d(p1, p2); },
      distarg, poly2polyhelp);
  def(
      "distance", +[](const CompoundPolygon2d& p1, const ConstPolygon2d& p2) { return lg::distance2d(p1, p2); },
      distarg, poly2polyhelp);
  def(
      "distance", +[](const CompoundPolygon2d& p1, const CompoundPolygon2d& p2) { return lg::distance2d(p1, p2); },
      distarg, poly2polyhelp);
  def(
      "distance", +[](const CompoundPolygon2d& p1, const ConstHybridPolygon2d& p2) { return lg::distance2d(p1, p2); },
      distarg, poly2polyhelp);
  def(
      "distance", +[](const ConstPolygon2d& p1, const ConstHybridPolygon2d& p2) { return lg::distance2d(p1, p2); },
      distarg, poly2polyhelp);

  // p2llt
  const auto* p2llthelp =
      "Shortest distance in 2D between a point and the polygon formed by a lanelet (regulatoryElements excluded)";
  const auto* llt2phelp =
      "Shortest distance in 2D between the polygon formed by a lanelet (regulatoryElements excluded) and a point ";
  def("distance", lg::distance2d<ConstLanelet, BasicPoint2d>, distarg, llt2phelp);
  def("distance", lg::distance2d<BasicPoint2d, ConstLanelet>, distarg, p2llthelp);
  def("distance", lg::distance2d<ConstPoint2d, ConstLanelet>, distarg, llt2phelp);
  def("distance", lg::distance2d<ConstLanelet, ConstPoint2d>, distarg, p2llthelp);

  // llt2llt
  def(
      "distance", +[](const ConstLanelet& llt2, const ConstLanelet& llt1) { return lg::distance2d(llt1, llt2); },
      (arg("lanelet1"), arg("lanelet2")), "Shortest distance in 2D between the polygons formed by two lanelets");

  // p2area
  const auto* p2arhelp =
      "Shortest distance in 2D between a point and the polygon formed by an Area (RegulatoryElements excluded)";
  const auto* ar2phelp =
      "Shortest distance in 2D between the polygon formed by an Area (RegulatoryElements excluded) and a point ";
  def("distance", lg::distance2d<ConstArea, BasicPoint2d>, distarg, ar2phelp);
  def("distance", lg::distance2d<BasicPoint2d, ConstArea>, distarg, p2arhelp);
  def("distance", lg::distance2d<ConstArea, ConstPoint2d>, distarg, ar2phelp);
  def("distance", lg::distance2d<ConstPoint2d, ConstArea>, distarg, p2arhelp);

  // area2area
  def("distance", lg::distance2d<ConstArea, ConstArea>, (arg("area1"), arg("area2")),
      "Shortest distance in 2D between the polygons formed by two areas (without their regulatory elements)");

  // 3d
  // p2p
  const auto* p2phelp3 = "3D distance between points";
  def("distance", lg::distance<ConstPoint3d, ConstPoint3d>, distarg, p2phelp3);
  def("distance", lg::distance<ConstPoint3d, BasicPoint3d>, distarg, p2phelp3);
  def("distance", lg::distance<BasicPoint3d, ConstPoint3d>, distarg, p2phelp3);
  def("distance", lg::distance<BasicPoint3d, BasicPoint3d>, distarg, p2phelp3);

  // p2l
  const auto* p2lhelp3 = "Shortest distance between a point and a linestring in 3D";
  const auto* l2phelp3 = "Shortest distance between a linestring and a point in 3D";
  def("distance", lg::distance<ConstPoint3d, HybridLs3d>, distarg, p2lhelp3);
  def("distance", lg::distance<HybridLs3d, ConstPoint3d>, distarg, l2phelp3);
  def("distance", lg::distance<ConstPoint3d, CompoundLineString3d>, distarg, p2lhelp3);
  def("distance", lg::distance<CompoundLineString3d, ConstPoint3d>, distarg, l2phelp3);
  def("distance", lg::distance<ConstPoint3d, ConstLineString3d>, distarg, p2lhelp3);
  def("distance", lg::distance<ConstLineString3d, ConstPoint3d>, distarg, l2phelp3);
  def("distance", lg::distance<ConstPoint3d, CompoundLineString3d>, distarg, p2lhelp3);
  def("distance", lg::distance<CompoundLineString3d, ConstPoint3d>, distarg, l2phelp3);

  // p2lines
  const auto distlinearg = (arg("point"), arg("linestrings"));
  const auto* p2lineshelp = "Shortest distance from a point to a list of linestrings";
  def("distanceToLines", distancePointToLss<ConstPoint2d>, distlinearg, p2lineshelp);
  def("distanceToLines", distancePointToLss<BasicPoint2d>, distlinearg, p2lineshelp);

  // l2l
  const auto* l2lhelp3 = "Shortest distance between two linestrings in 3D";
  def("distance", lg::distance3d<ConstLineString3d, ConstLineString3d>, distlinearg, l2lhelp3);
  def("distance", lg::distance3d<HybridLs3d, HybridLs3d>, distlinearg, l2lhelp3);
  def("distance", lg::distance3d<CompoundLineString3d, CompoundLineString3d>, distlinearg, l2lhelp3);
  def("distance", lg::distance3d<CompoundLineString3d, CompoundLineString3d>, distlinearg, l2lhelp3);
  def("distance", lg::distance3d<CompoundLineString3d, ConstLineString3d>, distlinearg, l2lhelp3);
  def("distance", lg::distance3d<ConstLineString3d, CompoundLineString3d>, distlinearg, l2lhelp3);
  def("distance", lg::distance3d<HybridLs3d, CompoundLineString3d>, distlinearg, l2lhelp3);
  def("distance", lg::distance3d<CompoundLineString3d, HybridLs3d>, distlinearg, l2lhelp3);
  def("distance", lg::distance3d<HybridLs3d, ConstLineString3d>, distlinearg, l2lhelp3);
  def("distance", lg::distance3d<ConstLineString3d, HybridLs3d>, distlinearg, l2lhelp3);

  // p2llt
  const auto* p2llthelp3 =
      "Shortest distance in 3D between a point and the polygon formed by a lanelet (regulatoryElements excluded)";
  const auto* llt2phelp3 =
      "Shortest distance in 3D between the polygon formed by a lanelet (regulatoryElements excluded) and a point ";
  def("distance", lg::distance3d<ConstLanelet, BasicPoint3d>, distarg, llt2phelp3);
  def("distance", lg::distance3d<BasicPoint3d, ConstLanelet>, distarg, p2llthelp3);

  // p2area
  const auto* p2arhelp3 =
      "Shortest distance in 3D between a point and the polygon formed by an Area (RegulatoryElements excluded)";
  const auto* ar2phelp3 =
      "Shortest distance in 3D between the polygon formed by an Area (RegulatoryElements excluded) and a point ";
  def("distance", lg::distance3d<ConstArea, BasicPoint3d>, distarg, ar2phelp3);
  def("distance", lg::distance3d<BasicPoint3d, ConstArea>, distarg, p2arhelp3);
  def("distance", lg::distance3d<ConstArea, ConstPoint3d>, distarg, ar2phelp3);
  def("distance", lg::distance3d<ConstPoint3d, ConstArea>, distarg, p2arhelp3);

  // equals 2d
  const auto equalarg = (arg("primitive1"), arg("primitive2"));
  const auto* pequalhelp = "Tests if two points are geometrically equivalent (up to a small margin) in 2D";
  // point
  def("equals", boost::geometry::equals<BasicPoint2d, BasicPoint2d>, equalarg, pequalhelp);
  def("equals", boost::geometry::equals<ConstPoint2d, ConstPoint2d>, equalarg, pequalhelp);
  def("equals", boost::geometry::equals<ConstPoint2d, BasicPoint2d>, equalarg, pequalhelp);

  // linestring
  const auto* lsequalhelp = "Tests if two linestrings are geometrically equivalent (up to a small margin) in 2D";
  def("equals", equalsLs<ConstLineString2d, ConstLineString2d>, equalarg, lsequalhelp);
  def("equals", equalsLs<CompoundLineString2d, CompoundLineString2d>, equalarg, lsequalhelp);
  def("equals", equalsLs<ConstLineString2d, CompoundLineString2d>, equalarg, lsequalhelp);
  def("equals", equalsLs<CompoundLineString2d, ConstLineString2d>, equalarg, lsequalhelp);

  // equals 3d
  const auto* pequalhelp3 = "Tests if two points are geometrically equivalent (up to a small margin) in 3D";
  // point
  def("equals", boost::geometry::equals<BasicPoint3d, BasicPoint3d>, equalarg, pequalhelp3);
  def("equals", boost::geometry::equals<ConstPoint3d, ConstPoint3d>, equalarg, pequalhelp3);
  def("equals", boost::geometry::equals<ConstPoint3d, BasicPoint3d>, equalarg, pequalhelp3);

  // bounding box
  const auto* boxhelp = "Get the surrounding axis-aligned bounding box in 2D";
  const auto boxarg = arg("primitive");
  def("boundingBox2d", boundingBox2dFor<ConstPoint2d>, boxarg, boxhelp);
  def("boundingBox2d", boundingBox2dFor<ConstLineString2d>, boxarg, boxhelp);
  def("boundingBox2d", boundingBox2dFor<ConstHybridLineString2d>, boxarg, boxhelp);
  def("boundingBox2d", boundingBox2dFor<ConstPolygon2d>, boxarg, boxhelp);
  def("boundingBox2d", boundingBox2dFor<ConstHybridPolygon2d>, boxarg, boxhelp);
  def("boundingBox2d", boundingBox2dFor<ConstLanelet>, boxarg, boxhelp);
  def("boundingBox2d", boundingBox2dFor<ConstArea>, boxarg, boxhelp);
  def("boundingBox2d", boundingBox2dFor<RegulatoryElementPtr>, boxarg, boxhelp);
  def("boundingBox2d", boundingBox2dFor<RegulatoryElementConstPtr>, boxarg, boxhelp);
  def("boundingBox2d", boundingBox2dFor<CompoundLineString2d>, boxarg, boxhelp);

  const auto* boxhelp3 = "Get the surrounding axis-aligned bounding box in 3D";
  def("boundingBox3d", boundingBox3dFor<ConstPoint3d>, boxarg, boxhelp3);
  def("boundingBox3d", boundingBox3dFor<ConstLineString3d>, boxarg, boxhelp3);
  def("boundingBox3d", boundingBox3dFor<ConstHybridLineString3d>, boxarg, boxhelp3);
  def("boundingBox3d", boundingBox3dFor<ConstPolygon3d>, boxarg, boxhelp3);
  def("boundingBox3d", boundingBox3dFor<ConstHybridPolygon3d>, boxarg, boxhelp3);
  def("boundingBox3d", boundingBox3dFor<ConstLanelet>, boxarg, boxhelp3);
  def("boundingBox3d", boundingBox3dFor<ConstArea>, boxarg, boxhelp3);
  def("boundingBox3d", boundingBox3dFor<RegulatoryElementPtr>, boxarg, boxhelp3);
  def("boundingBox3d", boundingBox3dFor<RegulatoryElementConstPtr>, boxarg, boxhelp3);
  def("boundingBox3d", boundingBox3dFor<CompoundLineString3d>, boxarg, boxhelp3);

  // area
  const auto areaargs = arg("polygon");
  const auto* areahelp = "Determine the area of a polygon";
  def("area", boost::geometry::area<BasicPolygon2d>, areaargs, areahelp);
  def("area", boost::geometry::area<ConstHybridPolygon2d>, areaargs, areahelp);
  def(
      "area", +[](const ConstArea& area) { boost::geometry::area(area.basicPolygonWithHoles2d()); }, arg("area"),
      "Determine the area of an Area, excluding its holes");

  // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
  auto makeArcCoordinates = static_cast<std::shared_ptr<ArcCoordinates> (*)(double, double)>(+[](double l, double d) {
    return std::make_shared<ArcCoordinates>(ArcCoordinates{l, d});
  });
  class_<ArcCoordinates, std::shared_ptr<ArcCoordinates>>("ArcCoordinates", "Coordinates along an arc", no_init)
      .def("__init__",
           make_constructor(makeArcCoordinates, default_call_policies(), (arg("length") = 0., arg("distance") = 0.)),
           "Create ArcCoordinates")
      .def_readwrite("length", &ArcCoordinates::length, "length along arc")
      .def_readwrite("distance", &ArcCoordinates::distance, "signed distance to arc (left is positive")
      .def(
          "__repr__", +[](ArcCoordinates ac) {
            return "ArcCoordinates(length=" + std::to_string(ac.length) + ", distance=" + std::to_string(ac.distance) +
                   ")";
          });

  const auto arcarg = (arg("linestring"), arg("point"));
  def("toArcCoordinates", lg::toArcCoordinates<ConstLineString2d>, arcarg,
      "Project a point into arc coordinates of the linestring");
  def("toArcCoordinates", lg::toArcCoordinates<CompoundLineString2d>, arcarg,
      "Project a point into arc coordinates of the linestring");

  def("fromArcCoordinates", lg::fromArcCoordinates<ConstLineString2d>, arcarg,
      "Create a point from arc coordinates of the linestring");
  def("fromArcCoordinates", lg::fromArcCoordinates<CompoundLineString2d>, arcarg,
      "Create a point from arc coordinates of the linestring");

  const auto* lenhelp = "Length of a linestring in 2D";
  const auto* lenhelp3 = "Length of a linestring in 3D";
  const auto lenarg = arg("linestring");
  def("length", lg::length<ConstLineString2d>, lenarg, lenhelp);
  def("length", lg::length<ConstLineString3d>, lenarg, lenhelp3);
  def("length", lg::length<CompoundLineString2d>, lenarg, lenhelp);
  def("length", lg::length<CompoundLineString3d>, lenarg, lenhelp3);

  const auto* interphelp = "Returns a point at a given distance along a linestring";
  const auto interparg = (arg("linestring"), arg("distance"));
  def("interpolatedPointAtDistance", lg::interpolatedPointAtDistance<BasicLineString2d>, interparg, interphelp);
  def("interpolatedPointAtDistance", lg::interpolatedPointAtDistance<BasicLineString3d>, interparg, interphelp);
  def("interpolatedPointAtDistance", lg::interpolatedPointAtDistance<ConstLineString2d>, interparg, interphelp);
  def("interpolatedPointAtDistance", lg::interpolatedPointAtDistance<ConstLineString3d>, interparg, interphelp);
  def("interpolatedPointAtDistance", lg::interpolatedPointAtDistance<CompoundLineString2d>, interparg, interphelp);
  def("interpolatedPointAtDistance", lg::interpolatedPointAtDistance<CompoundLineString3d>, interparg, interphelp);

  const auto* nearesthelp =
      "Returns the nearest point of the linestring linestring at a given distance (without interpolating)";
  const auto nearestarg = (arg("linestring"), arg("point"));
  def("nearestPointAtDistance", lg::nearestPointAtDistance<ConstLineString2d>, nearestarg, nearesthelp);
  def("nearestPointAtDistance", lg::nearestPointAtDistance<ConstLineString3d>, nearestarg, nearesthelp);
  def("nearestPointAtDistance", lg::nearestPointAtDistance<CompoundLineString2d>, nearestarg, nearesthelp);
  def("nearestPointAtDistance", lg::nearestPointAtDistance<CompoundLineString3d>, nearestarg, nearesthelp);

  const auto* projhelp2 = "Returns the point of the linestring that minimizes the distance to the given point in 2D";
  const auto* projhelp3 = "Returns the point of the linestring that minimizes the distance to the given point in 3D";
  const auto projArg = (arg("linestring"), arg("point"));
  def("project", lg::project<ConstLineString2d>, projArg, projhelp2);
  def("project", lg::project<ConstLineString3d>, projArg, projhelp3);
  def("project", lg::project<CompoundLineString2d>, projArg, projhelp2);
  def("project", lg::project<CompoundLineString3d>, projArg, projhelp3);

  const auto* projpHelp = "Returns the 3D points of the two linestrings that minimize the distance to each other in 3D";
  const auto projpArg = (arg("linestring1"), arg("linestring2"));
  def("projectedPoint3d", lg::projectedPoint3d<ConstLineString3d>, projpArg, projpHelp);
  def("projectedPoint3d", lg::projectedPoint3d<ConstHybridLineString3d>, projpArg, projpHelp);
  def("projectedPoint3d", lg::projectedPoint3d<CompoundLineString3d>, projpArg, projpHelp);

  const auto* inters2dHelp = "Returns whether two primitives intersect in 2D";
  const auto intersl2lArg = (arg("linestring1"), arg("linestring2"));
  def(
      "intersects2d",
      +[](const ConstLineString2d& ls1, const ConstLineString2d& ls2) {
        return lg::intersects(utils::toHybrid(ls1), utils::toHybrid(ls2));
      },
      intersl2lArg, inters2dHelp);
  def("intersects2d", lg::intersects<ConstHybridLineString2d, ConstHybridLineString2d>, intersl2lArg, inters2dHelp);
  def(
      "intersects2d",
      +[](const CompoundLineString2d& ls1, const CompoundLineString2d& ls2) {
        return lg::intersects(utils::toHybrid(ls1), utils::toHybrid(ls2));
      },
      intersl2lArg, inters2dHelp);
  def(
      "intersects2d",
      +[](const ConstPolygon2d& ls1, const ConstPolygon2d& ls2) {
        return lg::intersects(utils::toHybrid(ls1), utils::toHybrid(ls2));
      },
      (arg("polygon1"), arg("polygon2")), inters2dHelp);
  def("intersects2d", lg::intersects<ConstHybridPolygon2d, ConstHybridPolygon2d>, intersl2lArg, inters2dHelp);
  def("intersects2d", lg::intersects<BoundingBox3d, BoundingBox3d>, intersl2lArg, inters2dHelp);
  def("intersects2d", lg::intersects2d<ConstLanelet, ConstLanelet>, intersl2lArg, inters2dHelp);
  def("intersects2d", lg::intersects2d<ConstArea, ConstArea>, intersl2lArg, inters2dHelp);

  const auto* inters3dHelp = "Returns whether two primitives intersect in 3D (and within the given height tolerance)";
  def("intersects3d", lg::intersects3d<ConstLineString3d>,
      (arg("linestring1"), arg("linestring2"), arg("heightTolerance") = 3.), inters3dHelp);
  def("intersects3d", lg::intersects<BoundingBox3d, BoundingBox3d>, (arg("boundingBox1"), arg("boundingBox2")),
      "Returns whether two bounding boxes intersect in 3D");
  def("intersects3d", lg::intersects3d<ConstHybridLineString3d>,
      (arg("linestring1"), arg("linestring2"), arg("heightTolerance") = 3.), inters3dHelp);
  def("intersects3d", lg::intersects3d<CompoundLineString3d>,
      (arg("linestring1"), arg("linestring2"), arg("heightTolerance") = 3.), inters3dHelp);
  def("intersects3d", lg::intersects3d<ConstLanelet, ConstLanelet>,
      "Returns whether two lanelets intersect (touch or area  >0) in 3D or their distance in z is < heightTolerance",
      (arg("lanelet1"), arg("lanelet2"), arg("heightTolerance") = 3.));

  def("inside", lg::inside<ConstLanelet>, (arg("lanelet"), arg("point")), "Tests whether a point is within a lanelet");
  def("length2d", lg::length2d<ConstLanelet>, arg("lanelet"), "Calculate length of lanelet based on centerline in 2D");
  def("approximatedLength2d", lg::approximatedLength2d<ConstLanelet>, arg("lanelet"),
      "Approximate length by sampling points along left bound");
  def("length3d", lg::length3d<ConstLanelet>, "Calculate length of centerline in 3d", arg("lanelet"));
  def("distanceToCenterline2d", lg::distanceToCenterline2d<ConstLanelet>, (arg("lanelet"), arg("point")),
      "Returns the distance of a point to the centerline of a lanelet in 2D");
  def("distanceToCenterline3d", lg::distanceToCenterline3d<ConstLanelet>, (arg("lanelet"), arg("point")),
      "Returns the distance of a point to the centerline of a lanelet in 3D");
  def("overlaps2d", lg::overlaps2d<ConstLanelet, ConstLanelet>, (arg("lanelet1"), arg("lanelet2")),
      "Returns true if shared area of two lanelets is >0");
  def("overlaps3d", lg::overlaps3d<ConstLanelet, ConstLanelet>,
      "Approximates if two lanelets overlap (area  >0) in 3D or "
      "their distance in z is < heightTolerance",
      (arg("lanelet1"), arg("lanelet2"), arg("heightTolerance") = 3.));
  def(
      "intersectCenterlines2d",
      +[](const ConstLanelet& ll1, const ConstLanelet& ll2) {
        return toBasicVector(lg::intersectCenterlines2d(ll1, ll2));
      },
      (arg("lanelet1"), arg("lanelet2")),
      "Returns the intersection (a list of points) of the centerlines of two lanelets in 2D");

  def("leftOf", lg::leftOf<ConstLanelet, ConstLanelet>, "Returns if first lanelet is directly left of second",
      (arg("lanelet1"), arg("lanelet2")));
  def("rightOf", lg::rightOf<ConstLanelet, ConstLanelet>, "Returns if first lanelet is directly right of second",
      (arg("lanelet1"), arg("lanelet2")));
  def("follows", lg::follows<ConstLanelet, ConstLanelet>, "Returns if first lanelet precedes the second",
      (arg("lanelet1"), arg("lanelet2")));

  wrapFindNearest<Point3d>();
  wrapFindNearest<LineString3d>();
  wrapFindNearest<Polygon3d>();
  wrapFindNearest<Lanelet>();
  wrapFindNearest<Area>();
  wrapFindNearest<RegulatoryElementPtr>();

  // find within, point layer
  wrapFindWithin2d<Point3d, Point2d>();
  wrapFindWithin2d<Point3d, BasicPoint2d>();
  wrapFindWithin2d<Point3d, BoundingBox2d>();
  wrapFindWithin2d<Point3d, Polygon2d>();
  wrapFindWithin2d<Point3d, BasicPolygon2d>();
  wrapFindWithin2d<Point3d, LineString2d>();
  wrapFindWithin2d<Point3d, BasicLineString2d>();
  wrapFindWithin2d<Point3d, CompoundLineString2d>();
  wrapFindWithin2d<Point3d, Lanelet>();
  wrapFindWithin2d<Point3d, Area>();
  wrapFindWithin3d<Point3d, Point3d>();
  wrapFindWithin3d<Point3d, BasicPoint3d>();
  wrapFindWithin3d<Point3d, BoundingBox3d>();
  wrapFindWithin3d<Point3d, Polygon3d>();
  wrapFindWithin3d<Point3d, BasicPolygon3d>();
  wrapFindWithin3d<Point3d, LineString3d>();
  wrapFindWithin3d<Point3d, BasicLineString3d>();
  wrapFindWithin3d<Point3d, CompoundLineString3d>();
  wrapFindWithin3d<Point3d, Lanelet>();
  wrapFindWithin3d<Point3d, Area>();

  // linestring layer
  wrapFindWithin2d<LineString3d, Point2d>();
  wrapFindWithin2d<LineString3d, BasicPoint2d>();
  wrapFindWithin2d<LineString3d, BoundingBox2d>();
  wrapFindWithin2d<LineString3d, Polygon2d>();
  wrapFindWithin2d<LineString3d, BasicPolygon2d>();
  wrapFindWithin2d<LineString3d, LineString2d>();
  wrapFindWithin2d<LineString3d, Lanelet>();
  wrapFindWithin2d<LineString3d, Area>();
  wrapFindWithin2d<LineString3d, BasicLineString2d>();
  wrapFindWithin2d<LineString3d, CompoundLineString2d>();
  wrapFindWithin3d<LineString3d, Point3d>();
  wrapFindWithin3d<LineString3d, BasicPoint3d>();

  // polygon layer
  wrapFindWithin2d<Polygon3d, Point2d>();
  wrapFindWithin2d<Polygon3d, BasicPoint2d>();
  wrapFindWithin2d<Polygon3d, BoundingBox2d>();
  wrapFindWithin2d<Polygon3d, Polygon2d>();
  wrapFindWithin2d<Polygon3d, BasicPolygon2d>();
  wrapFindWithin2d<Polygon3d, LineString2d>();
  wrapFindWithin2d<Polygon3d, BasicLineString2d>();
  wrapFindWithin2d<Polygon3d, CompoundLineString2d>();
  wrapFindWithin2d<Polygon3d, Lanelet>();
  wrapFindWithin2d<Polygon3d, Area>();
  wrapFindWithin3d<Polygon3d, Point3d>();
  wrapFindWithin3d<Polygon3d, BasicPoint3d>();

  // lanelet layer
  wrapFindWithin2d<Lanelet, Point2d>();
  wrapFindWithin2d<Lanelet, BasicPoint2d>();
  wrapFindWithin2d<Lanelet, BoundingBox2d>();
  wrapFindWithin2d<Lanelet, Polygon2d>();
  wrapFindWithin2d<Lanelet, BasicPolygon2d>();
  wrapFindWithin2d<Lanelet, LineString2d>();
  wrapFindWithin2d<Lanelet, BasicLineString2d>();
  wrapFindWithin2d<Lanelet, CompoundLineString2d>();
  wrapFindWithin2d<Lanelet, Lanelet>();
  wrapFindWithin2d<Lanelet, Area>();
  wrapFindWithin3d<Lanelet, Point3d>();
  wrapFindWithin3d<Lanelet, BasicPoint3d>();

  // area layer
  wrapFindWithin2d<Area, Point2d>();
  wrapFindWithin2d<Area, BasicPoint2d>();
  wrapFindWithin2d<Area, BoundingBox2d>();
  wrapFindWithin2d<Area, Polygon2d>();
  wrapFindWithin2d<Area, BasicPolygon2d>();
  wrapFindWithin2d<Area, LineString2d>();
  wrapFindWithin2d<Area, BasicLineString2d>();
  wrapFindWithin2d<Area, CompoundLineString2d>();
  wrapFindWithin2d<Area, Lanelet>();
  wrapFindWithin2d<Area, Area>();
  wrapFindWithin3d<Area, Point3d>();
  wrapFindWithin3d<Area, BasicPoint3d>();

  // boost::geometry functions for convenience
  const auto* intershelp = "Returns the list of points where two linestrings intersect";
  const auto intersarg = (arg("linestring1"), arg("linestring2"));
  def(
      "intersection",
      +[](const BasicLineString2d& ls1, const BasicLineString2d& ls2) {
        BasicPoints2d pts;
        boost::geometry::intersection(ls1, ls2, pts);
        return toBasicVector(pts);
      },
      intersarg, intershelp);

  def(
      "intersection",
      +[](const CompoundLineString2d& ls1, const ConstLineString2d& ls2) {
        BasicPoints2d pts;
        boost::geometry::intersection(ls1.basicLineString(), ls2.basicLineString(), pts);
        return toBasicVector(pts);
      },
      intersarg, intershelp);

  def(
      "intersection",
      +[](const CompoundLineString2d& ls1, const CompoundLineString2d& ls2) {
        BasicPoints2d pts;
        boost::geometry::intersection(ls1.basicLineString(), ls2.basicLineString(), pts);
        return toBasicVector(pts);
      },
      intersarg, intershelp);

  def(
      "intersection",
      +[](const ConstLineString2d& ls1, const ConstLineString2d& ls2) {
        BasicPoints2d pts;
        boost::geometry::intersection(ls1.basicLineString(), ls2.basicLineString(), pts);
        return toBasicVector(pts);
      },
      intersarg, intershelp);
}
