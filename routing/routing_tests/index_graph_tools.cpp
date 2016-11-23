#include "routing/routing_tests/index_graph_tools.hpp"

#include "testing/testing.hpp"

#include "routing/car_model.hpp"

namespace routing_test
{
using namespace routing;

void TestGeometryLoader::Load(uint32_t featureId, RoadGeometry & road) const
{
  auto it = m_roads.find(featureId);
  if (it == m_roads.cend())
    return;

  road = it->second;
}

void TestGeometryLoader::AddRoad(uint32_t featureId, bool oneWay,
                                 RoadGeometry::Points const & points)
{
  auto it = m_roads.find(featureId);
  if (it != m_roads.end())
  {
    ASSERT(false, ("Already contains feature", featureId));
    return;
  }

  m_roads[featureId] = RoadGeometry(oneWay, 1.0 /* speed */, points);
}

Joint MakeJoint(vector<RoadPoint> const & points)
{
  Joint joint;
  for (auto const & point : points)
    joint.AddPoint(point);

  return joint;
}

shared_ptr<EdgeEstimator> CreateEstimator()
{
  return CreateCarEdgeEstimator(make_shared<CarModelFactory>()->GetVehicleModel());
}

AStarAlgorithm<IndexGraph>::Result CalculateRoute(IndexGraph & graph,
                                                  RoadPoint const & start, RoadPoint const & finish,
                                                  vector<RoadPoint> & roadPoints)
{
  AStarAlgorithm<IndexGraph> algorithm;
  RoutingResult<Joint::Id> routingResult;
  AStarAlgorithm<IndexGraph>::Result const resultCode =  algorithm.FindPath(
      graph, graph.InsertJoint(start), graph.InsertJoint(finish), routingResult, {}, {});

  graph.RedressRoute(routingResult.path, roadPoints);
  return resultCode;
}

void TestRouteSegments(IndexGraph & graph, RoadPoint const & start, RoadPoint const & finish,
                       AStarAlgorithm<IndexGraph>::Result expectedRouteResult,
                       vector<RoadPoint> const & expectedRoute)
{
  vector<RoadPoint> route;
  AStarAlgorithm<IndexGraph>::Result const resultCode = CalculateRoute(graph, start, finish, route);
  TEST_EQUAL(resultCode, expectedRouteResult, ());
  TEST_EQUAL(route, expectedRoute, ());
}

void TestRouteGeometry(RoadPoint const & start, RoadPoint const & finish,
                       AStarAlgorithm<IndexGraph>::Result expectedRouteResult,
                       vector<m2::PointD> const & expectedRouteGeom, IndexGraph & graph)
{
  vector<RoadPoint> route;
  AStarAlgorithm<IndexGraph>::Result const resultCode = CalculateRoute(graph, start, finish, route);
  TEST_EQUAL(resultCode, expectedRouteResult, ());
  TEST_EQUAL(route.size(), expectedRouteGeom.size(), ());
  for (size_t i = 0; i < route.size(); ++i)
  {
    RoadGeometry roadGeom = graph.GetRoad(route[i].GetFeatureId());
    CHECK_LESS(route[i].GetPointId(), roadGeom.GetPointsCount(), ());
    TEST_EQUAL(expectedRouteGeom[i], roadGeom.GetPoint(route[i].GetPointId()), ());
  }
}
}  // namespace routing_test
