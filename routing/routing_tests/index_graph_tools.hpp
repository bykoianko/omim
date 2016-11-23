#pragma once

#include "routing/base/astar_algorithm.hpp"
#include "routing/edge_estimator.hpp"
#include "routing/index_graph.hpp"

#include "std/algorithm.hpp"
#include "std/shared_ptr.hpp"
#include "std/unique_ptr.hpp"
#include "std/unordered_map.hpp"
#include "std/vector.hpp"

namespace routing_test
{
class TestGeometryLoader final : public routing::GeometryLoader
{
public:
  // GeometryLoader overrides:
  void Load(uint32_t featureId, routing::RoadGeometry & road) const override;

  void AddRoad(uint32_t featureId, bool oneWay, routing::RoadGeometry::Points const & points);

private:
  unordered_map<uint32_t, routing::RoadGeometry> m_roads;
};

routing::Joint MakeJoint(vector<routing::RoadPoint> const & points);

shared_ptr<routing::EdgeEstimator> CreateEstimator();

routing::AStarAlgorithm<routing::IndexGraph>::Result CalculateRoute(routing::IndexGraph & graph,
    routing::RoadPoint const & start, routing::RoadPoint const & finish,
    vector<routing::RoadPoint> & roadPoints);

void TestRouteSegments(routing::IndexGraph & graph, routing::RoadPoint const & start,
                       routing::RoadPoint const & finish,
                       routing::AStarAlgorithm<routing::IndexGraph>::Result expectedRouteResult,
                       vector<routing::RoadPoint> const & expectedRoute);
}  // namespace routing_test
