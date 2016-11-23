#include "testing/testing.hpp"

#include "routing/routing_tests/index_graph_tools.hpp"

#include "routing/car_model.hpp"

#include "std/unique_ptr.hpp"
#include "std/vector.hpp"

namespace
{
using namespace routing;
using namespace routing_test;

void TestRoutes(vector<RoadPoint> const & starts,
                vector<RoadPoint> const & finishes,
                vector<vector<RoadPoint>> const & expectedRoutes,
                IndexGraph & graph)
{
  CHECK_EQUAL(starts.size(), expectedRoutes.size(), ());
  CHECK_EQUAL(finishes.size(), expectedRoutes.size(), ());

  for (size_t i = 0; i < expectedRoutes.size(); ++i)
  {
     TestRouteSegments(graph, starts[i], finishes[i],
                       AStarAlgorithm<IndexGraph>::Result::OK, expectedRoutes[i]);
  }
}

void EdgeTest(Joint::Id vertex, size_t expectedIntgoingNum, size_t expectedOutgoingNum,
              IndexGraph & graph)
{
  vector<IndexGraph::TEdgeType> ingoing;
  graph.GetIngoingEdgesList(vertex, ingoing);
  TEST_EQUAL(ingoing.size(), expectedIntgoingNum, ());

  vector<IndexGraph::TEdgeType> outgoing;
  graph.GetOutgoingEdgesList(vertex, outgoing);
  TEST_EQUAL(outgoing.size(), expectedOutgoingNum, ());
};

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
// Note. F0, F1 and F2 are one segment features. F3 is a two segments feature.
unique_ptr<IndexGraph> BuildTriangularGraph()
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

  unique_ptr<IndexGraph> graph = make_unique<IndexGraph>(
        loader(), CreateCarEdgeEstimator(make_shared<CarModelFactory>()->GetVehicleModel()));
  graph->Import(joints, {} /* restrictions */);
  return graph;
}

// Route through triangular graph without any restrictions.
UNIT_TEST(TriangularGraph)
{
  RoadPoint const kStart(2, 0);
  RoadPoint const kFinish(1, 1);
  unique_ptr<IndexGraph> graph = BuildTriangularGraph();
  vector<RoadPoint> const expectedRoute = {{2 /* feature id */, 0 /* seg id */},
                                           {2, 1}, {1, 1}};
  TestRouteSegments(*graph, kStart, kFinish, AStarAlgorithm<IndexGraph>::Result::OK, expectedRoute);
}

// Route through triangular graph with feature 2 disabled.
UNIT_TEST(TriangularGraph_DisableF2)
{
  RoadPoint const kStart(2, 0);
  RoadPoint const kFinish(1, 1);
  unique_ptr<IndexGraph> graph = BuildTriangularGraph();
  graph->DisableEdge(graph->GetJointIdForTesting({2 /* feature id */, 0 /* point id */}),
                     graph->GetJointIdForTesting({2, 1}));

  vector<RoadPoint> const expectedRouteOneEdgeRemoved = {{3 /* feature id */, 0 /* seg id */},
                                                         {3, 1}, {3, 2}, {0, 1}};
  TestRouteSegments(*graph, kStart, kFinish, AStarAlgorithm<IndexGraph>::Result::OK,
                    expectedRouteOneEdgeRemoved);
}

// Route through triangular graph with restriction type no from feature 2 to feature 1.
UNIT_TEST(TriangularGraph_RestrictionNoF2F1)
{
  RoadPoint const kStart(2, 0);
  RoadPoint const kFinish(1, 1);
  unique_ptr<IndexGraph> graph = BuildTriangularGraph();
  graph->ApplyRestrictionNo({2 /* feature id */, 1 /* seg id */}, {1, 0},
                            graph->GetJointIdForTesting({1, 0}));

  vector<RoadPoint> const expectedRouteRestrictionF2F1No = {{3 /* feature id */, 0 /* seg id */},
                                                            {3, 1}, {3, 2}, {0, 1}};
  TestRouteSegments(*graph, kStart, kFinish, AStarAlgorithm<IndexGraph>::Result::OK,
                    expectedRouteRestrictionF2F1No);
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
// Note. F1 is a two setments feature. The others are one setment ones.
unique_ptr<IndexGraph> BuildCornerGraph()
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

  unique_ptr<IndexGraph> graph = make_unique<IndexGraph>(
        loader(), CreateCarEdgeEstimator(make_shared<CarModelFactory>()->GetVehicleModel()));
  graph->Import(joints, {} /* restrictions */);
  return graph;
}

// Route through corner graph without any restrictions.
UNIT_TEST(CornerGraph)
{
  RoadPoint const kStart(1, 0);
  RoadPoint const kFinish(0, 1);
  unique_ptr<IndexGraph> graph = BuildCornerGraph();

  // Route along F1 and F0.
  vector<RoadPoint> const expectedRoute = {{1 /* feature id */, 0 /* point id */},
                                           {1, 1}, {1, 2}, {0, 1}};
  TestRouteSegments(*graph, kStart, kFinish, AStarAlgorithm<IndexGraph>::Result::OK, expectedRoute);
}

// Generate geometry based on feature 1 of corner graph.
UNIT_TEST(CornerGraph_CreateFakeFeature1Geometry)
{
  unique_ptr<IndexGraph> graph = BuildCornerGraph();
  graph->InsertJoint({1 /* feature id */, 0 /* point id */}); // Start joint.
  graph->InsertJoint({0 /* feature id */, 1 /* point id */}); // Finish joint.

  vector<RoadPoint> roadPoints;
  graph->CreateRoadPointsList(graph->GetJointIdForTesting({1 /* feature id */, 0 /* point id */}),
                              graph->GetJointIdForTesting({1, 2}), roadPoints);
  vector<RoadPoint> const expectedDirectOrder = {{1, 0}, {1, 1}, {1, 2}};
  TEST_EQUAL(roadPoints, expectedDirectOrder, ());
  RoadGeometry geometryDirect;
  graph->CreateFakeFeatureGeometry(roadPoints, geometryDirect);
  RoadGeometry expectedGeomentryDirect(true /* one way */, 1.0 /* speed */,
                                       buffer_vector<m2::PointD, 32>({{2.0, 0.0}, {1.0, 0.0},
                                                                     {0.0, 0.0}}));
  TEST_EQUAL(geometryDirect, expectedGeomentryDirect, ());
}

// Generate geometry based on reversed feature 1 of corner graph.
UNIT_TEST(CornerGraph_CreateFakeReversedFeature1Geometry)
{
  unique_ptr<IndexGraph> graph = BuildCornerGraph();
  graph->InsertJoint({1 /* feature id */, 0 /* point id */}); // Start joint.
  graph->InsertJoint({0 /* feature id */, 1 /* point id */}); // Finish joint.

  vector<RoadPoint> roadPoints;
  graph->CreateRoadPointsList(graph->GetJointIdForTesting({1 /* feature id */, 2 /* point id */}),
                              graph->GetJointIdForTesting({1, 0}), roadPoints);
  vector<RoadPoint> const expectedBackOrder = {{1, 2}, {1, 1}, {1, 0}};
  TEST_EQUAL(roadPoints, expectedBackOrder, ());
  RoadGeometry geometryBack;
  graph->CreateFakeFeatureGeometry(roadPoints, geometryBack);
  RoadGeometry expectedGeomentryBack(true /* one way */, 1.0 /* speed */,
                                     buffer_vector<m2::PointD, 32>({{0.0, 0.0}, {1.0, 0.0},
                                                                    {2.0, 0.0}}));
  TEST_EQUAL(geometryBack, expectedGeomentryBack, ());
}

// Route through corner graph with adding a fake edge.
UNIT_TEST(CornerGraph_AddFakeFeature)
{
  RoadPoint const kStart(1, 0);
  RoadPoint const kFinish(0, 1);
  unique_ptr<IndexGraph> graph = BuildCornerGraph();
  graph->InsertJoint({1 /* feature id */, 0 /* point id */}); // Start joint.
  graph->InsertJoint({0 /* feature id */, 1 /* point id */}); // Finish joint.

  graph->AddFakeFeature(graph->GetJointIdForTesting(kStart), graph->GetJointIdForTesting(kFinish),
                        {} /* viaPointGeometry */);

  vector<RoadPoint> const expectedRouteByFakeFeature = {{IndexGraph::kStartFakeFeatureIds, 0 /* seg id */},
                                                        {IndexGraph::kStartFakeFeatureIds, 1 /* seg id */}};
  TestRouteSegments(*graph, kStart, kFinish, AStarAlgorithm<IndexGraph>::Result::OK, expectedRouteByFakeFeature);
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
// Note. F1 and F2 are two segments features. The others are one segment ones.
unique_ptr<IndexGraph> BuildTwoSquaresGraph()
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

  unique_ptr<IndexGraph> graph = make_unique<IndexGraph>(
        loader(), CreateCarEdgeEstimator(make_shared<CarModelFactory>()->GetVehicleModel()));
  graph->Import(joints, {} /* restrictions */);
  return graph;
}

// Route through two squares graph without any restrictions.
UNIT_TEST(TwoSquaresGraph)
{
  unique_ptr<IndexGraph> graph = BuildTwoSquaresGraph();

  vector<RoadPoint> const starts = {{2 /* feature id */, 0 /* point id */}, {2, 0}, {2, 0}};
  vector<RoadPoint> const finishes = {{6 /* feature id */, 0 /* point id */}, {6, 1}, {5, 1}};
  vector<RoadPoint> const expectedRoute0 = {{2 /* feature id */, 0 /* point id */},
                                            {2, 1}, {2, 2}};
  vector<RoadPoint> const expectedRoute1 = {{2 /* feature id */, 0 /* point id */},
                                            {2, 1}, {2, 2}, {6, 1}};
  vector<RoadPoint> const expectedRoute2 = {{2 /* feature id */, 0 /* point id */},
                                            {2, 1}, {2, 2}, {6, 1}, {5, 1}};

  TestRoutes(starts, finishes, {expectedRoute0, expectedRoute1, expectedRoute2}, *graph);
}

// Tests on method IndexGraph::GetIntermediatePoints().
UNIT_TEST(TwoSquaresGraph_GetIntermediatePoints)
{
  unique_ptr<IndexGraph> graph = BuildTwoSquaresGraph();

  vector<Joint::Id> const jointsF3 = {graph->GetJointIdForTesting({3 /* feature id */, 0 /* point id */}),
                                      graph->GetJointIdForTesting({3, 1})};
  vector<RoadPoint> roadPointsF3;
  vector<size_t> internalJointIdxF3;
  graph->GetIntermediatePoints(jointsF3, roadPointsF3, internalJointIdxF3);
  TEST(roadPointsF3.empty(), ());
  TEST(internalJointIdxF3.empty(), ());

  vector<Joint::Id> const jointsF1 = {graph->GetJointIdForTesting({1 /* feature id */, 0 /* point id */}),
                                      graph->GetJointIdForTesting({1, 2})};
  vector<RoadPoint> roadPointsF1;
  vector<size_t> internalJointIdxF1;
  graph->GetIntermediatePoints(jointsF1, roadPointsF1, internalJointIdxF1);
  vector<RoadPoint> const expectedRoadPointsF1 = {{1, 1}};
  TEST_EQUAL(expectedRoadPointsF1, roadPointsF1, ());
  TEST(internalJointIdxF1.empty(), ());

  vector<Joint::Id> const jointsF3F1 = {graph->GetJointIdForTesting({3 /* feature id */, 0 /* point id */}),
                                        graph->GetJointIdForTesting({3, 1}),
                                        graph->GetJointIdForTesting({1, 2})};
  vector<RoadPoint> roadPointsF3F1;
  vector<size_t> internalJointIdxF3F1;
  graph->GetIntermediatePoints(jointsF3F1, roadPointsF3F1, internalJointIdxF3F1);
  vector<RoadPoint> const expectedRoadPointsF3F1 = {{1, 0}, {1, 1}};
  vector<size_t> const expectedInternalJointIdxF1 = {0 /* Intermediate joint index. */};
  TEST_EQUAL(expectedRoadPointsF3F1, roadPointsF3F1, ());
  TEST_EQUAL(internalJointIdxF3F1, expectedInternalJointIdxF1, ());
}

// Route through two squares graph with adding a Fake-0 edge.
UNIT_TEST(TwoSquaresGraph_AddFakeFeatureZero)
{
  unique_ptr<IndexGraph> graph = BuildTwoSquaresGraph();
  graph->AddFakeFeature(graph->InsertJoint({2 /* feature id */, 1 /* point id */}),
                        graph->GetJointIdForTesting({6 /* feature id */, 1 /* point id */}), {} /* via points */);

  vector<RoadPoint> const starts = {{2 /* feature id */, 0 /* point id */}, {2, 0}, {2, 0}};
  vector<RoadPoint> const finishes = {{6 /* feature id */, 0 /* point id */}, {6, 1}, {5, 1}};
  vector<RoadPoint> const expectedRoute0 = {{2 /* feature id */, 0 /* point id */},
                                            {2, 1}, {2, 2}};
  vector<RoadPoint> const expectedRoute1 = {{2 /* feature id */, 0 /* point id */},
                                            {2, 1}, {IndexGraph::kStartFakeFeatureIds, 1}};
  vector<RoadPoint> const expectedRoute2 = {{2 /* feature id */, 0 /* point id */},
                                            {2, 1}, {IndexGraph::kStartFakeFeatureIds, 1}, {5, 1}};

  TestRoutes(starts, finishes, {expectedRoute0, expectedRoute1, expectedRoute2}, *graph);
}

// Route through two squares graph with adding a Fake-0, Fake-1 and Fake-2 edge.
UNIT_TEST(TwoSquaresGraph_AddFakeFeatureZeroOneTwo)
{
  unique_ptr<IndexGraph> graph = BuildTwoSquaresGraph();
  // Adding features: Fake 0, Fake 1 and Fake 2.
  graph->AddFakeFeature(graph->InsertJoint({2 /* feature id */, 1 /* point id */}),
                        graph->GetJointIdForTesting({6 /* feature id */, 1 /* point id */}), {} /* via points */);
  graph->AddFakeFeature(graph->InsertJoint({1 /* feature id */, 1 /* point id */}),
                        graph->GetJointIdForTesting({5 /* feature id */, 1 /* point id */}), {} /* via points */);
  graph->AddFakeFeature(graph->GetJointIdForTesting({2 /* feature id */, 0 /* point id */}),
                        graph->GetJointIdForTesting({1 /* feature id */, 1 /* point id */}),
                        {} /* via points */);

  vector<RoadPoint> const starts = {{2 /* feature id */, 0 /* point id */}, {2, 0}, {2, 0}};
  vector<RoadPoint> const finishes = {{6 /* feature id */, 0 /* point id */}, {6, 1}, {5, 1}};
  vector<RoadPoint> const expectedRoute0 = {{2 /* feature id */, 0 /* point id */},
                                            {2, 1}, {2, 2}};
  vector<RoadPoint> const expectedRoute1 = {{2 /* feature id */, 0 /* point id */},
                                            {2, 1}, {IndexGraph::kStartFakeFeatureIds, 1}};
  vector<RoadPoint> const expectedRoute2 = {{IndexGraph::kStartFakeFeatureIds + 2 /* Fake 2 */, 0},
                                            {IndexGraph::kStartFakeFeatureIds + 2 /* Fake 2 */, 1},
                                            {IndexGraph::kStartFakeFeatureIds + 1 /* Fake 1 */, 1}};
  TestRoutes(starts, finishes, {expectedRoute0, expectedRoute1, expectedRoute2}, *graph);

  // Disabling Fake-2 feature.
  graph->DisableEdge(graph->GetJointIdForTesting({IndexGraph::kStartFakeFeatureIds + 2 /* Fake 2 */, 0}),
                     graph->GetJointIdForTesting({IndexGraph::kStartFakeFeatureIds + 2 /* Fake 2 */, 1}));
  vector<RoadPoint> const expectedRoute2Disable2 = {{2 /* feature id */, 0 /* point id */},
                                                    {2, 1}, {IndexGraph::kStartFakeFeatureIds, 1}, {5, 1}};
  TestRoutes(starts, finishes, {expectedRoute0, expectedRoute1, expectedRoute2Disable2}, *graph);
}

//      Finish
// 1 *-F4-*-F5-*
//   |         |
//   F2        F3
//   |         |
// 0 *---F1----*---F0---* Start
//   0         1        2
// Note 1. All features are two-way. (It's possible to move along any direction of the features.)
// Note 2. Any feature contains of one segment.
unique_ptr<IndexGraph> BuildFlagGraph()
{
  auto const loader = []()
  {
    unique_ptr<TestGeometryLoader> loader = make_unique<TestGeometryLoader>();
    loader->AddRoad(0 /* feature id */, false /* one way */, buffer_vector<m2::PointD, 32>(
                           {{2.0, 0.0}, {1.0, 0.0}}));
    loader->AddRoad(1 /* feature id */, false /* one way */, buffer_vector<m2::PointD, 32>(
                           {{1.0, 0.0}, {0.0, 0.0}}));
    loader->AddRoad(2 /* feature id */, false /* one way */, buffer_vector<m2::PointD, 32>(
                           {{0.0, 0.0}, {0.0, 1.0}}));
    loader->AddRoad(3 /* feature id */, false /* one way */, buffer_vector<m2::PointD, 32>(
                           {{1.0, 0.0}, {1.0, 1.0}}));
    loader->AddRoad(4 /* feature id */, false /* one way */, buffer_vector<m2::PointD, 32>(
                           {{0.0, 1.0}, {0.5, 1.0}}));
    loader->AddRoad(5 /* feature id */, false /* one way */, buffer_vector<m2::PointD, 32>(
                           {{0.5, 1.0}, {1.0, 1.0}}));
    return loader;
  };

  vector<Joint> const joints = {
    MakeJoint({{1 /* feature id */, 1 /* point id */}, {2, 0}}), /* joint at point (0, 0) */
    MakeJoint({{2, 1}, {4, 0}}), /* joint at point (0, 1) */
    MakeJoint({{4, 1}, {5, 0}}), /* joint at point (0.5, 1) */
    MakeJoint({{1, 0}, {3, 0}, {0, 1}}), /* joint at point (1, 0) */
    MakeJoint({{3, 1}, {5, 1}}), /* joint at point (1, 1) */
  };

  unique_ptr<IndexGraph> graph = make_unique<IndexGraph>(
        loader(), CreateCarEdgeEstimator(make_shared<CarModelFactory>()->GetVehicleModel()));
  graph->Import(joints, {} /* restrictions */);
  return graph;
}

// @TODO(bykoianko) It's necessary to implement put several no restriction on the same junction
// ingoing edges of the same jucntion. For example test with flag graph with two restriction
// type no from F0 to F3 and form F0 to F1.

// Route through flag graph without any restrictions.
UNIT_TEST(FlagGraph)
{
  unique_ptr<IndexGraph> graph = BuildFlagGraph();

  RoadPoint const kStart(0 /* feature id */, 0 /* point id */);
  RoadPoint const kFinish(5, 0);
  vector<RoadPoint> const expectedRoute = {{0 /* feature id */, 0 /* point id */},
                                           {0, 1}, {3, 1}, {5, 0}};

  TestRouteSegments(*graph, kStart, kFinish, AStarAlgorithm<IndexGraph>::Result::OK, expectedRoute);
}

// Route through flag graph with one restriciton (type no) from F0 to F3.
UNIT_TEST(FlagGraph_RestrictionF0F3No)
{
  RoadPoint const kStart(0 /* feature id */, 0 /* point id */);
  RoadPoint const kFinish(5, 0);
  unique_ptr<IndexGraph> graph = BuildFlagGraph();
  graph->InsertJoint(kStart);
  Joint::Id const restictionCenterId = graph->GetJointIdForTesting({0, 1});

  // Testing outgoing and ingoing edge number near restriction joint.
  EdgeTest(restictionCenterId, 3 /* expectedIntgoingNum */, 3 /* expectedOutgoingNum */, *graph);
  graph->ApplyRestrictionNo({0 /* feature id */, 1 /* point id */}, {3 /* feature id */, 0 /* point id */},
                            restictionCenterId);
  EdgeTest(restictionCenterId, 2 /* expectedIntgoingNum */, 3 /* expectedOutgoingNum */, *graph);

  // Testing route building after adding the restriction.
  vector<RoadPoint> const expectedRoute = {{IndexGraph::kStartFakeFeatureIds, 0 /* point id */},
                                           {IndexGraph::kStartFakeFeatureIds, 1},
                                           {IndexGraph::kStartFakeFeatureIds, 2},
                                           {2 /* feature id */, 1 /* point id */},
                                           {4, 1}};

  TestRouteSegments(*graph, kStart, kFinish, AStarAlgorithm<IndexGraph>::Result::OK, expectedRoute);
}

// Route through flag graph with one restriciton (type only) from F0 to F1.
UNIT_TEST(FlagGraph_RestrictionF0F1Only)
{
  RoadPoint const kStart(0 /* feature id */, 0 /* point id */);
  RoadPoint const kFinish(5, 0);
  unique_ptr<IndexGraph> graph = BuildFlagGraph();
  graph->InsertJoint(kStart);

  Joint::Id const restictionCenterId = graph->GetJointIdForTesting({0, 1});
  graph->ApplyRestrictionOnly({0 /* feature id */, 1 /* point id */}, {1 /* feature id */, 0 /* point id */},
                             restictionCenterId);

  vector<RoadPoint> const expectedRoute = {{IndexGraph::kStartFakeFeatureIds, 0 /* point id */},
                                           {IndexGraph::kStartFakeFeatureIds, 1},
                                           {IndexGraph::kStartFakeFeatureIds, 2},
                                           {2 /* feature id */, 1 /* point id */},
                                           {4, 1}};
  TestRouteSegments(*graph, kStart, kFinish, AStarAlgorithm<IndexGraph>::Result::OK, expectedRoute);
}
}  // namespace
