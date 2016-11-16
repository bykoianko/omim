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

class TestGeometryLoader : public GeometryLoader
{
public:
  // GeometryLoader overrides:
  void Load(uint32_t featureId, RoadGeometry & road) const override;

  void AddRoad(uint32_t featureId, buffer_vector<m2::PointD, 32> const & points);

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

void TestGeometryLoader::AddRoad(uint32_t featureId, buffer_vector<m2::PointD, 32> const & points)
{
  auto it = m_roads.find(featureId);
  if (it != m_roads.end())
  {
    ASSERT(false, ("Already contains feature", featureId));
    return;
  }

  m_roads[featureId] = RoadGeometry(false, 1.0 /* speed */, points);
}

Joint MakeJoint(vector<FtPoint> const & points)
{
  Joint joint;
  for (auto const & point : points)
    joint.AddEntry(point);

  return joint;
}

shared_ptr<EdgeEstimator> CreateEstimator()
{
  return CreateCarEdgeEstimator(make_shared<CarModelFactory>()->GetVehicleModel());
}

void TestRoute(IndexGraph & graph, FtPoint const & start, FtPoint const & finish,
               size_t expectedLength)
{
  LOG(LINFO, ("Test route", start.GetFeatureId(), ",", start.GetPointId(), "=>",
              finish.GetFeatureId(), ",", finish.GetPointId()));

  AStarAlgorithm<IndexGraph> algorithm;
  RoutingResult<Joint::Id> routingResult;

  AStarAlgorithm<IndexGraph>::Result const resultCode = algorithm.FindPath(
      graph, graph.InsertJoint(start), graph.InsertJoint(finish), routingResult, {}, {});
  vector<FtPoint> const & ftPoints = graph.RedressRoute(routingResult.path);

  TEST_EQUAL(resultCode, AStarAlgorithm<IndexGraph>::Result::OK, ());
  TEST_EQUAL(ftPoints.size(), expectedLength, ());
}

uint32_t AbsDelta(uint32_t v0, uint32_t v1) { return v0 > v1 ? v0 - v1 : v1 - v0; }
}  // namespace

namespace routing_test
{
//  Roads     R1:
//
//            -2
//            -1
//  R0: -2 -1  0  1  2
//             1
//             2
//
UNIT_TEST(FindPathCross)
{
  unique_ptr<TestGeometryLoader> loader = make_unique<TestGeometryLoader>();
  loader->AddRoad(0 /* featureId */,
                  buffer_vector<m2::PointD, 32>(
                      {{-2.0, 0.0}, {-1.0, 0.0}, {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}}));
  loader->AddRoad(1 /* featureId */,
                  buffer_vector<m2::PointD, 32>(
                      {{0.0, -2.0}, {-1.0, 0.0}, {0.0, 0.0}, {0.0, 1.0}, {0.0, 2.0}}));

  IndexGraph graph(move(loader), CreateEstimator());

  graph.Import({MakeJoint({{0, 2}, {1, 2}})});

  vector<FtPoint> points;
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
    buffer_vector<m2::PointD, 32> street;
    buffer_vector<m2::PointD, 32> avenue;
    for (uint32_t j = 0; j < kCitySize; ++j)
    {
      street.emplace_back(static_cast<double>(i), static_cast<double>(j));
      avenue.emplace_back(static_cast<double>(j), static_cast<double>(i));
    }
    loader->AddRoad(i, street);
    loader->AddRoad(i + kCitySize, avenue);
  }

  IndexGraph graph(move(loader), CreateEstimator());

  vector<Joint> joints;
  for (uint32_t i = 0; i < kCitySize; ++i)
  {
    for (uint32_t j = 0; j < kCitySize; ++j)
      joints.emplace_back(MakeJoint({{i, j}, {j + kCitySize, i}}));
  }
  graph.Import(joints);

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
}  // namespace routing_test
