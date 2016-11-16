#include "testing/testing.hpp"

#include "routing/base/astar_algorithm.hpp"
#include "routing/car_model.hpp"
#include "routing/edge_estimator.hpp"
#include "routing/index_graph.hpp"

#include "geometry/point2d.hpp"

#include "base/assert.hpp"

#include "std/algorithm.hpp"
#include "std/unique_ptr.hpp"
#include "std/unordered_map.hpp"
#include "std/vector.hpp"

namespace
{
using namespace routing;

class TestGeometryLoader final : public GeometryLoader
{
public:
  // GeometryLoader overrides:
  void Load(uint32_t featureId, RoadGeometry & road) const override;

  void AddRoad(uint32_t featureId, bool oneWay, RoadGeometry::Points const & points);

private:
  unordered_map<uint32_t, RoadGeometry> m_roads;
};

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

void TestRoute(IndexGraph & graph, RoadPoint const & start, RoadPoint const & finish,
               size_t expectedLength)
{
  vector<RoadPoint> route;
  AStarAlgorithm<IndexGraph>::Result const resultCode = CalculateRoute(graph, start, finish, route);

  TEST_EQUAL(resultCode, AStarAlgorithm<IndexGraph>::Result::OK, ());
  TEST_EQUAL(route.size(), expectedLength, ());
}

void TestRouteSegments(IndexGraph & graph, RoadPoint const & start, RoadPoint const & finish,
                       AStarAlgorithm<IndexGraph>::Result expectedRouteResult,
                       vector<RoadPoint> const & expectedRoute)
{
  LOG(LINFO, ("Testing route segments", start.GetFeatureId(), ",", start.GetPointId(), "=>",
              finish.GetFeatureId(), ",", finish.GetPointId()));

  vector<RoadPoint> route;
  AStarAlgorithm<IndexGraph>::Result const resultCode = CalculateRoute(graph, start, finish, route);
  TEST_EQUAL(resultCode, expectedRouteResult, ());
  TEST_EQUAL(route, expectedRoute, ());
}

void TestEdges(IndexGraph const & graph, Joint::Id jointId,
               vector<Joint::Id> const & expectedTargets, bool forward)
{
  vector<JointEdge> edges;
  if (forward)
    graph.GetOutgoingEdgesList(jointId, edges);
  else
    graph.GetIngoingEdgesList(jointId, edges);

  vector<Joint::Id> targets;
  for (JointEdge const & edge : edges)
    targets.push_back(edge.GetTarget());

  sort(targets.begin(), targets.end());

  ASSERT_EQUAL(targets, expectedTargets, ());
}

void TestOutgoingEdges(IndexGraph const & graph, Joint::Id jointId,
                       vector<Joint::Id> const & expectedTargets)
{
  TestEdges(graph, jointId, expectedTargets, true);
}

void TestIngoingEdges(IndexGraph const & graph, Joint::Id jointId,
                      vector<Joint::Id> const & expectedTargets)
{
  TestEdges(graph, jointId, expectedTargets, false);
}

uint32_t AbsDelta(uint32_t v0, uint32_t v1) { return v0 > v1 ? v0 - v1 : v1 - v0; }
}  // namespace

//                   R4 (one way down)
//
// R1     J2--------J3         -1
//         ^         v
//         ^         v
// R0 *---J0----*---J1----*     0
//         ^         v
//         ^         v
// R2     J4--------J5          1
//
//        R3 (one way up)       y
//
// x: 0    1    2    3    4
//
UNIT_TEST(EdgesTest)
{
  unique_ptr<TestGeometryLoader> loader = make_unique<TestGeometryLoader>();
  loader->AddRoad(
      0 /* featureId */, false,
      RoadGeometry::Points({{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}, {4.0, 0.0}}));
  loader->AddRoad(1 /* featureId */, false, RoadGeometry::Points({{1.0, -1.0}, {3.0, -1.0}}));
  loader->AddRoad(2 /* featureId */, false, RoadGeometry::Points({{1.0, -1.0}, {3.0, -1.0}}));
  loader->AddRoad(3 /* featureId */, true,
                  RoadGeometry::Points({{1.0, 1.0}, {1.0, 0.0}, {1.0, -1.0}}));
  loader->AddRoad(4 /* featureId */, true,
                  RoadGeometry::Points({{3.0, -1.0}, {3.0, 0.0}, {3.0, 1.0}}));

  IndexGraph graph(move(loader), CreateEstimator());

  vector<Joint> joints;
  joints.emplace_back(MakeJoint({{0, 1}, {3, 1}}));  // J0
  joints.emplace_back(MakeJoint({{0, 3}, {4, 1}}));  // J1
  joints.emplace_back(MakeJoint({{1, 0}, {3, 2}}));  // J2
  joints.emplace_back(MakeJoint({{1, 1}, {4, 0}}));  // J3
  joints.emplace_back(MakeJoint({{2, 0}, {3, 0}}));  // J4
  joints.emplace_back(MakeJoint({{2, 1}, {4, 2}}));  // J5
  graph.Import(joints, {});

  TestOutgoingEdges(graph, 0, {1, 2});
  TestOutgoingEdges(graph, 1, {0, 5});
  TestOutgoingEdges(graph, 2, {3});
  TestOutgoingEdges(graph, 3, {1, 2});
  TestOutgoingEdges(graph, 4, {0, 5});
  TestOutgoingEdges(graph, 5, {4});

  TestIngoingEdges(graph, 0, {1, 4});
  TestIngoingEdges(graph, 1, {0, 3});
  TestIngoingEdges(graph, 2, {0, 3});
  TestIngoingEdges(graph, 3, {2});
  TestIngoingEdges(graph, 4, {5});
  TestIngoingEdges(graph, 5, {1, 4});
}

namespace routing_test
{
//  Roads     R1:
//
//            -2
//            -1
//  R0  -2 -1  0  1  2
//             1
//             2
//
UNIT_TEST(FindPathCross)
{
  unique_ptr<TestGeometryLoader> loader = make_unique<TestGeometryLoader>();
  loader->AddRoad(
      0 /* featureId */, false,
      RoadGeometry::Points({{-2.0, 0.0}, {-1.0, 0.0}, {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}}));
  loader->AddRoad(
      1 /* featureId */, false,
      RoadGeometry::Points({{0.0, -2.0}, {-1.0, 0.0}, {0.0, 0.0}, {0.0, 1.0}, {0.0, 2.0}}));

  IndexGraph graph(move(loader), CreateEstimator());

  graph.Import({MakeJoint({{0, 2}, {1, 2}})}, {} /* restrictions */);

  vector<RoadPoint> points;
  for (uint32_t i = 0; i < 5; ++i)
  {
    points.emplace_back(0, i);
    points.emplace_back(1, i);
  }

  for (auto const & start : points)
  {
    for (auto const & finish : points)
    {
      uint32_t expectedLength;
      // Length of the route is the number of route points.
      // Example: p0 --- p1 --- p2
      // 2 segments, 3 points,
      // Therefore route length = geometrical length + 1
      if (start.GetFeatureId() == finish.GetFeatureId())
        expectedLength = AbsDelta(start.GetPointId(), finish.GetPointId()) + 1;
      else
        expectedLength = AbsDelta(start.GetPointId(), 2) + AbsDelta(finish.GetPointId(), 2) + 1;
      TestRoute(graph, start, finish, expectedLength);
    }
  }
}

// Roads   R4  R5  R6  R7
//
//    R0   0 - * - * - *
//         |   |   |   |
//    R1   * - 1 - * - *
//         |   |   |   |
//    R2   * - * - 2 - *
//         |   |   |   |
//    R3   * - * - * - 3
//
UNIT_TEST(FindPathManhattan)
{
  uint32_t constexpr kCitySize = 4;
  unique_ptr<TestGeometryLoader> loader = make_unique<TestGeometryLoader>();
  for (uint32_t i = 0; i < kCitySize; ++i)
  {
    RoadGeometry::Points street;
    RoadGeometry::Points avenue;
    for (uint32_t j = 0; j < kCitySize; ++j)
    {
      street.emplace_back(static_cast<double>(i), static_cast<double>(j));
      avenue.emplace_back(static_cast<double>(j), static_cast<double>(i));
    }

    loader->AddRoad(i, false, street);
    loader->AddRoad(i + kCitySize, false, avenue);
  }

  IndexGraph graph(move(loader), CreateEstimator());

  vector<Joint> joints;
  for (uint32_t i = 0; i < kCitySize; ++i)
  {
    for (uint32_t j = 0; j < kCitySize; ++j)
      joints.emplace_back(MakeJoint({{i, j}, {j + kCitySize, i}}));
  }

  graph.Import(joints, {} /* restrictions */);

  for (uint32_t startY = 0; startY < kCitySize; ++startY)
  {
    for (uint32_t startX = 0; startX < kCitySize; ++startX)
    {
      for (uint32_t finishY = 0; finishY < kCitySize; ++finishY)
      {
        for (uint32_t finishX = 0; finishX < kCitySize; ++finishX)
          TestRoute(graph, {startX, startY}, {finishX, finishY},
                    AbsDelta(startX, finishX) + AbsDelta(startY, finishY) + 1);
      }
    }
  }
}

// Finish
// 2 *
//   ^ ↖
//   |   F1
//   |      ↖
// 1 |        *
//   F0         ↖
//   |            F2
//   |              ↖
// 0 *<--F3---<--F3---* Start
//   0        1       2
// Building or trying to build routes form Start point to Finish point
// * without any restrictions
// * without feature F2
// * without feature F2 and F3
// Note. F0, F1 and F2 are one segment features. F3 is a two segment feature.
UNIT_TEST(FindPathFourFeatureTriangularGraph)
{
  auto const loader = []()
  {
    unique_ptr<TestGeometryLoader> loader = make_unique<TestGeometryLoader>();
    loader->AddRoad(0 /* featureId */, true /* oneWay */, buffer_vector<m2::PointD, 32>(
                           {{0.0, 0.0}, {0.0, 2.0}}));
    loader->AddRoad(1 /* featureId */, true /* oneWay */, buffer_vector<m2::PointD, 32>(
                           {{1.0, 1.0}, {0.0, 2.0}}));
    loader->AddRoad(2 /* featureId */, true /* oneWay */, buffer_vector<m2::PointD, 32>(
                           {{0.0, 2.0}, {1.0, 1.0}}));
    loader->AddRoad(3 /* featureId */, true /* oneWay */, buffer_vector<m2::PointD, 32>(
                           {{2.0, 0.0}, {1.0, 0.0}, {0.0, 0.0}}));
    return loader;
  };

  vector<Joint> const joints = {MakeJoint({{2, 0}, {3, 0}}), /* joint at point (2, 0) */
                                MakeJoint({{3, 2}, {0, 0}}), /* joint at point (0, 0) */
                                MakeJoint({{2, 1}, {1, 0}}), /* joint at point (1, 1) */
                                MakeJoint({{0, 1}, {1, 1}})  /* joint at point (0, 2) */
                               };
  RoadPoint const kStart(2, 0);
  RoadPoint const kFinish(1, 1);

  // Disabling edges.
  {
    IndexGraph graph(loader(), CreateCarEdgeEstimator(make_shared<CarModelFactory>()->GetVehicleModel()));
    graph.Import(joints, {} /* restrictions */);
    vector<RoadPoint> const expectedRoute = {{2 /* feature id */, 0 /* seg id */},
                                             {2, 1}, {1, 1}};
    TestRouteSegments(graph, kStart, kFinish, AStarAlgorithm<IndexGraph>::Result::OK, expectedRoute);

    graph.DisableEdge(graph.GetJointIdForTesting({2 /* feature id */, 0 /* point id */}),
                      graph.GetJointIdForTesting({2, 1}));
    vector<RoadPoint> const expectedRouteOneEdgeRemoved = {{3 /* feature id */, 0 /* seg id */},
                                                           {3, 1}, {3, 2}, {0, 1}};
    TestRouteSegments(graph, kStart, kFinish, AStarAlgorithm<IndexGraph>::Result::OK,
                      expectedRouteOneEdgeRemoved);

    graph.DisableEdge(graph.GetJointIdForTesting({3 /* feature id */, 0 /* point id */}),
                      graph.GetJointIdForTesting({3, 2}));
    TestRouteSegments(graph, kStart, kFinish, AStarAlgorithm<IndexGraph>::Result::NoPath, {});
  }

  // Restriction "No".
  {
    IndexGraph graph(loader(), CreateCarEdgeEstimator(make_shared<CarModelFactory>()->GetVehicleModel()));
    graph.Import(joints, {} /* restrictions */);
    graph.ApplyRestrictionNo({2 /* feature id */, 1 /* seg id */}, {1, 0},
                             graph.GetJointIdForTesting({1, 0}));
    vector<RoadPoint> const expectedRouteRestrictionF2F1No = {{3 /* feature id */, 0 /* seg id */},
                                                              {3, 1}, {3, 2}, {0, 1}};
    TestRouteSegments(graph, kStart, kFinish, AStarAlgorithm<IndexGraph>::Result::OK,
                      expectedRouteRestrictionF2F1No);
  }
}

// Finish
// 2 *
//   ^ ↖
//   |   ↖
//   |     ↖
// 1 |       Fake adding one link feature
//   F0        ↖
//   |           ↖
//   |             ↖
// 0 *<--F1---<--F1--* Start
//   0        1       2
// Note. F1 is a two link feature. The others are one link ones.
UNIT_TEST(FindPathAddingOneLinkFakeFeature)
{
  auto const loader = []()
  {
    unique_ptr<TestGeometryLoader> loader = make_unique<TestGeometryLoader>();
    loader->AddRoad(0 /* feature id */, true /* oneWay */, buffer_vector<m2::PointD, 32>(
                           {{0.0, 0.0}, {0.0, 2.0}}));
    loader->AddRoad(1 /* feature id */, true /* oneWay */, buffer_vector<m2::PointD, 32>(
                           {{2.0, 0.0}, {1.0, 0.0}, {0.0, 0.0}}));
    return loader;
  };

  vector<Joint> const joints = {MakeJoint({{1, 2}, {0, 0}}), /* joint at point (0, 0) */};

  RoadPoint const kStart(1, 0);
  RoadPoint const kFinish(0, 1);

  IndexGraph graph(loader(), CreateCarEdgeEstimator(make_shared<CarModelFactory>()->GetVehicleModel()));
  graph.Import(joints, {} /* restrictions */);

  // Route along F1 and F0.
  vector<RoadPoint> const expectedRoute = {{1 /* feature id */, 0 /* point id */},
                                           {1, 1}, {1, 2}, {0, 1}};
  TestRouteSegments(graph, kStart, kFinish, AStarAlgorithm<IndexGraph>::Result::OK, expectedRoute);

  // Creating route point list test.
  // Note. It's used in the test that joints for start and finish are added while routing before.
  vector<RoadPoint> roadPoints;
  graph.CreateRoadPointsList(graph.GetJointIdForTesting({1, 0}),
                             graph.GetJointIdForTesting({1, 2}), roadPoints);
  vector<RoadPoint> const expectedDirectOrder = {{1, 0}, {1, 1}, {1, 2}};
  TEST_EQUAL(roadPoints, expectedDirectOrder, ());
  RoadGeometry geometryDirect;
  graph.CreateFakeFeatureGeometry(roadPoints, geometryDirect);
  RoadGeometry expectedGeomentryDirect(true /* one way */, 1.0 /* speed */,
                                       buffer_vector<m2::PointD, 32>({{2.0, 0.0}, {1.0, 0.0},
                                                                     {0.0, 0.0}}));
  TEST_EQUAL(geometryDirect, expectedGeomentryDirect, ());

  graph.CreateRoadPointsList(graph.GetJointIdForTesting({1 /* feature id */, 2 /* point id */}),
                             graph.GetJointIdForTesting({1, 0}), roadPoints);
  vector<RoadPoint> const expectedBackOrder = {{1, 2}, {1, 1}, {1, 0}};
  TEST_EQUAL(roadPoints, expectedBackOrder, ());
  RoadGeometry geometryBack;
  graph.CreateFakeFeatureGeometry(roadPoints, geometryBack);
  RoadGeometry expectedGeomentryBack(true /* one way */, 1.0 /* speed */,
                                     buffer_vector<m2::PointD, 32>({{0.0, 0.0}, {1.0, 0.0},
                                                                    {2.0, 0.0}}));
  TEST_EQUAL(geometryBack, expectedGeomentryBack, ());

  // Adding a shortcut fake feature and building route.
  graph.AddFakeFeature(graph.GetJointIdForTesting(kStart), graph.GetJointIdForTesting(kFinish),
                       {} /* viaPointGeometry */);
  vector<RoadPoint> const expectedRouteByFakeFeature = {{IndexGraph::kStartFakeFeatureIds, 0 /* seg id */},
                                                        {IndexGraph::kStartFakeFeatureIds, 1 /* seg id */}};
  TestRouteSegments(graph, kStart, kFinish, AStarAlgorithm<IndexGraph>::Result::OK, expectedRouteByFakeFeature);
}

// Finish 2   Finish 1  Finish 0
// 2 *<---F5----*<---F6---*
//   ^ ↖       ^ ↖       ^
//   | Fake-1   | Fake-0  |
//   |     ↖   F1    ↖   F2
//   |       ↖ |       ↖ |
// 1 F0         *          *
//   |          ^  ↖      ^
//   |         F1  Fake-2 F2
//   |          |       ↖ |
// 0 *<----F4---*<---F3----* Start
//   0          1          2
// Note. F1 and F2 are two link features. The others are one link ones.
// This test tests the following things
// * Building all three routes from Start to Finish0, Finish1 and Finish 2
// * Adding Fake-0 and building all the routes
// * Then adding Fake-1 and Fake-2 and building all the routes
// * Disabling Fake-2 and building all the routes
UNIT_TEST(FindPathAddingThreeOneLinkFakeFeatures)
{
  auto const loader = []()
  {
    unique_ptr<TestGeometryLoader> loader = make_unique<TestGeometryLoader>();
    loader->AddRoad(0 /* feature id */, true /* oneWay */, buffer_vector<m2::PointD, 32>(
                           {{0.0, 0.0}, {0.0, 2.0}}));
    loader->AddRoad(1 /* feature id */, true /* oneWay */, buffer_vector<m2::PointD, 32>(
                           {{1.0, 0.0}, {1.0, 1.0}, {1.0, 2.0}}));
    loader->AddRoad(2 /* feature id */, true /* oneWay */, buffer_vector<m2::PointD, 32>(
                           {{2.0, 0.0}, {2.0, 1.0}, {2.0, 2.0}}));
    loader->AddRoad(3 /* feature id */, true /* oneWay */, buffer_vector<m2::PointD, 32>(
                           {{2.0, 0.0}, {1.0, 0.0}}));
    loader->AddRoad(4 /* feature id */, true /* oneWay */, buffer_vector<m2::PointD, 32>(
                           {{1.0, 0.0}, {0.0, 0.0}}));
    loader->AddRoad(5 /* feature id */, true /* oneWay */, buffer_vector<m2::PointD, 32>(
                           {{1.0, 2.0}, {0.0, 2.0}}));
    loader->AddRoad(6 /* feature id */, true /* oneWay */, buffer_vector<m2::PointD, 32>(
                           {{2.0, 2.0}, {1.0, 2.0}}));
    return loader;
  };

  vector<Joint> const joints = {
    MakeJoint({{4 /* featureId */, 1 /* pointId */}, {0, 0}}), /* joint at point (0, 0) */
    MakeJoint({{0, 1}, {5, 1}}), /* joint at point (0, 2) */
    MakeJoint({{4, 0}, {1, 0}, {3, 1}}), /* joint at point (1, 0) */
    MakeJoint({{5, 0}, {1, 2}, {6, 1}}), /* joint at point (1, 2) */
    MakeJoint({{3, 0}, {2, 0}}), /* joint at point (2, 0) */
    MakeJoint({{2, 2}, {6, 0}}), /* joint at point (2, 2) */
  };

  RoadPoint const kStart(2 /* feature id */, 0 /* point id */);
  RoadPoint const kFinish0(6, 0);
  RoadPoint const kFinish1(6, 1);
  RoadPoint const kFinish2(5, 1);

  auto const testRoutes = [&](IndexGraph & graph, vector<RoadPoint> const & expectedRoute0,
      vector<RoadPoint> const & expectedRoute1, vector<RoadPoint> const & expectedRoute2)
  {
    TestRouteSegments(graph, kStart, kFinish0, AStarAlgorithm<IndexGraph>::Result::OK, expectedRoute0);
    TestRouteSegments(graph, kStart, kFinish1, AStarAlgorithm<IndexGraph>::Result::OK, expectedRoute1);
    TestRouteSegments(graph, kStart, kFinish2, AStarAlgorithm<IndexGraph>::Result::OK, expectedRoute2);
  };

  // Routing without fake edges.
  IndexGraph graph(loader(), CreateCarEdgeEstimator(make_shared<CarModelFactory>()->GetVehicleModel()));
  graph.Import(joints, {} /* restrictions */);

  vector<RoadPoint> const expectedRoute0 = {{2 /* feature id */, 0 /* point id */},
                                            {2, 1}, {2, 2}};
  vector<RoadPoint> const expectedRoute1 = {{2 /* feature id */, 0 /* point id */},
                                            {2, 1}, {2, 2}, {6, 1}};
  vector<RoadPoint> const expectedRoute2 = {{2 /* feature id */, 0 /* point id */},
                                            {2, 1}, {2, 2}, {6, 1}, {5, 1}};
  testRoutes(graph, expectedRoute0, expectedRoute1, expectedRoute2);

  // Getting intermediate points.
  vector<Joint::Id> jointsF3 = {graph.GetJointIdForTesting({3 /* feature id */, 0 /* point id */}),
                                graph.GetJointIdForTesting({3, 1})};
  vector<RoadPoint> roadPointsF3;
  vector<size_t> internalJointIdxF3;
  graph.GetIntermediatePoints(jointsF3, roadPointsF3, internalJointIdxF3);
  TEST(roadPointsF3.empty(), ());
  TEST(internalJointIdxF3.empty(), ());

  vector<Joint::Id> jointsF1 = {graph.GetJointIdForTesting({1 /* feature id */, 0}),
                                graph.GetJointIdForTesting({1, 2})};
  vector<RoadPoint> roadPointsF1;
  vector<size_t> internalJointIdxF1;
  graph.GetIntermediatePoints(jointsF1, roadPointsF1, internalJointIdxF1);
  vector<RoadPoint> const expectedRoadPointsF1 = {{1, 1}};
  TEST_EQUAL(expectedRoadPointsF1, roadPointsF1, ());
  TEST(internalJointIdxF1.empty(), ());

  vector<Joint::Id> jointsF3F1 = {graph.GetJointIdForTesting({3 /* feature id */, 0 /* point id */}),
                                  graph.GetJointIdForTesting({3, 1}),
                                  graph.GetJointIdForTesting({1, 2})};
  vector<RoadPoint> roadPointsF3F1;
  vector<size_t> internalJointIdxF3F1;
  graph.GetIntermediatePoints(jointsF3F1, roadPointsF3F1, internalJointIdxF3F1);
  vector<RoadPoint> const expectedRoadPointsF3F1 = {{1, 0}, {1, 1}};
  vector<size_t> const expectedInternalJointIdxF1 = {0 /* Intermediate joint index. */};
  TEST_EQUAL(expectedRoadPointsF3F1, roadPointsF3F1, ());
  TEST_EQUAL(internalJointIdxF3F1, expectedInternalJointIdxF1, ());

  // Adding Fake-0 features.
  graph.AddFakeFeature(graph.InsertJoint({2 /* feature id */, 1 /* point id */}),
                       graph.GetJointIdForTesting({6 /* feature id */, 1 /* point id */}), {} /* via points */);
  vector<RoadPoint> const expectedRoute1Fake0 = {{2 /* feature id */, 0 /* point id */},
                                                 {2, 1}, {IndexGraph::kStartFakeFeatureIds, 1}};
  vector<RoadPoint> const expectedRoute2Fake0 = {{2 /* feature id */, 0 /* point id */},
                                                 {2, 1}, {IndexGraph::kStartFakeFeatureIds, 1}, {5, 1}};
  testRoutes(graph, expectedRoute0, expectedRoute1Fake0, expectedRoute2Fake0);

  // Adding Fake-1 and Fake-2 features.
  graph.AddFakeFeature(graph.InsertJoint({1 /* feature id */, 1 /* point id */}),
                       graph.GetJointIdForTesting({5 /* feature id */, 1 /* point id */}), {} /* via points */);
  graph.AddFakeFeature(graph.GetJointIdForTesting({2 /* feature id */, 0 /* point id */}),
                       graph.GetJointIdForTesting({1 /* feature id */, 1 /* point id */}),
                       {} /* via points */);
  vector<RoadPoint> const expectedRoute2Fake012 = {{IndexGraph::kStartFakeFeatureIds + 2 /* Fake 2 */, 0},
                                                   {IndexGraph::kStartFakeFeatureIds + 2 /* Fake 2 */, 1},
                                                   {IndexGraph::kStartFakeFeatureIds + 1 /* Fake 1 */, 1}};
  testRoutes(graph, expectedRoute0, expectedRoute1Fake0, expectedRoute2Fake012);

  // Disabling Fake-2 feature.
  graph.DisableEdge(graph.GetJointIdForTesting({IndexGraph::kStartFakeFeatureIds + 2 /* Fake 2 */, 0}),
                    graph.GetJointIdForTesting({IndexGraph::kStartFakeFeatureIds + 2 /* Fake 2 */, 1}));
  testRoutes(graph, expectedRoute0, expectedRoute1Fake0, expectedRoute2Fake0);
}

//      Finish
// 1 *-F4-*-F5-*
//   |         |
//   F2        F3
//   |         |
// 0 *---F1----*---F0---* Start
//   0         1        2
// The idea of the test to prohibit a turn from F0 to F3 and build a route from Start to Finish.
// All featrures are two-way. (It's possible to move along any direction of the features.)
UNIT_TEST(FindPathRestrictionNoAndOnlyComplicatedCase)
{
  auto const edgeTest = [](Joint::Id vertex, size_t expectedIntgoingNum, size_t expectedOutgoingNum,
                           IndexGraph & graph)
  {
    vector<IndexGraph::TEdgeType> ingoing;
    graph.GetIngoingEdgesList(vertex, ingoing);
    TEST_EQUAL(ingoing.size(), expectedIntgoingNum, ());

    vector<IndexGraph::TEdgeType> outgoing;
    graph.GetOutgoingEdgesList(vertex, outgoing);
    TEST_EQUAL(outgoing.size(), expectedOutgoingNum, ());
  };

  auto const loader = []()
  {
    unique_ptr<TestGeometryLoader> loader = make_unique<TestGeometryLoader>();
    loader->AddRoad(0 /* featureId */, false /* oneWay */, buffer_vector<m2::PointD, 32>(
                           {{2.0, 0.0}, {1.0, 0.0}}));
    loader->AddRoad(1 /* featureId */, false /* oneWay */, buffer_vector<m2::PointD, 32>(
                           {{1.0, 0.0}, {0.0, 0.0}}));
    loader->AddRoad(2 /* featureId */, false /* oneWay */, buffer_vector<m2::PointD, 32>(
                           {{0.0, 0.0}, {0.0, 1.0}}));
    loader->AddRoad(3 /* featureId */, false /* oneWay */, buffer_vector<m2::PointD, 32>(
                           {{1.0, 0.0}, {1.0, 1.0}}));
    loader->AddRoad(4 /* featureId */, false /* oneWay */, buffer_vector<m2::PointD, 32>(
                           {{0.0, 1.0}, {0.5, 1.0}}));
    loader->AddRoad(5 /* featureId */, false /* oneWay */, buffer_vector<m2::PointD, 32>(
                           {{0.5, 1.0}, {1.0, 1.0}}));
    return loader;
  };

  vector<Joint> const joints = {
    MakeJoint({{1 /* featureId */, 1 /* pointId */}, {2, 0}}), /* joint at point (0, 0) */
    MakeJoint({{2, 1}, {4, 0}}), /* joint at point (0, 1) */
    MakeJoint({{4, 1}, {5, 0}}), /* joint at point (0.5, 1) */
    MakeJoint({{1, 0}, {3, 0}, {0, 1}}), /* joint at point (1, 0) */
    MakeJoint({{3, 1}, {5, 1}}), /* joint at point (1, 1) */
  };

  RoadPoint const kStart(0 /* featureId */, 0 /* pointId */);
  RoadPoint const kFinish(5, 0);

  vector<RoadPoint> const expectedRoute = {{0 /* feature id */, 0 /* point id */},
                                            {0, 1}, {3, 1}, {5, 0}};

  // Restriction no test.
  {
    IndexGraph graph(loader(), CreateCarEdgeEstimator(make_shared<CarModelFactory>()->GetVehicleModel()));
    graph.Import(joints, {} /* restrictions */);
    Joint::Id const restictionCenterId = graph.GetJointIdForTesting({0, 1});

    // Route without restriction.
    // Note. While creating the route joints for |kStart| and for |kFinish| is being added to  |graph|.
    // It influences next tests in this scope.
    TestRouteSegments(graph, kStart, kFinish, AStarAlgorithm<IndexGraph>::Result::OK, expectedRoute);

    // Route with one no restriciton from F0 to F3.
    edgeTest(restictionCenterId, 3 /* expectedIntgoingNum */, 3 /* expectedOutgoingNum */, graph);
    graph.ApplyRestrictionNo({0 /* feature id */, 1 /* point id */}, {3 /* feature id */, 0 /* point id */},
                             restictionCenterId);
    edgeTest(restictionCenterId, 2 /* expectedIntgoingNum */, 3 /* expectedOutgoingNum */, graph);

    vector<RoadPoint> const expectedLongRoute = {{IndexGraph::kStartFakeFeatureIds, 0 /* point id */},
                                                 {IndexGraph::kStartFakeFeatureIds, 1},
                                                 {IndexGraph::kStartFakeFeatureIds, 2},
                                                 {2 /* feature id */, 1 /* point id */},
                                                 {4, 1}};
    TestRouteSegments(graph, kStart, kFinish, AStarAlgorithm<IndexGraph>::Result::OK, expectedLongRoute);

    // Route with one no restriciton from F0 to F3 and form F0 to F1.
    // @TODO(bykoianko) It's necessary to implement put several no restriction on the same junction
    // ingoing edges of the same jucntion.
  }

  // Restriction only test.
  {
    IndexGraph graph(loader(), CreateCarEdgeEstimator(make_shared<CarModelFactory>()->GetVehicleModel()));
    graph.Import(joints, {} /* restrictions */);
    Joint::Id const restictionCenterId = graph.GetJointIdForTesting({0, 1});

    // Route without restriction.
    // Note. While creating the route joints for |kStart| and for |kFinish| is being added to  |graph|.
    // It influences next tests in this scope.
    TestRouteSegments(graph, kStart, kFinish, AStarAlgorithm<IndexGraph>::Result::OK, expectedRoute);

    vector<RoadPoint> const expectedLongRoute = {{IndexGraph::kStartFakeFeatureIds, 0 /* point id */},
                                                 {IndexGraph::kStartFakeFeatureIds, 1},
                                                 {IndexGraph::kStartFakeFeatureIds, 2},
                                                 {2 /* feature id */, 1 /* point id */},
                                                 {4, 1}};
    graph.ApplyRestrictionOnly({0 /* feature id */, 1 /* point id */}, {1 /* feature id */, 0 /* point id */},
                               restictionCenterId);
    TestRouteSegments(graph, kStart, kFinish, AStarAlgorithm<IndexGraph>::Result::OK, expectedLongRoute);
  }
}
}  // namespace routing_test
