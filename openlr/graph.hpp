#pragma once

#include "routing/features_road_graph.hpp"
#include "routing/road_graph.hpp"

#include "routing_common/car_model.hpp"

#include "geometry/point2d.hpp"

#include <cstddef>
#include <map>
#include <memory>
#include <utility>
#include <vector>

class DataSource;

namespace openlr
{
// TODO(mgsergio): Inherit from FeaturesRoadGraph.
class Graph
{
public:
  using Edge = routing::Edge;
  using EdgeVector = routing::FeaturesRoadGraph::TEdgeVector;
  using Junction = routing::Junction;

  Graph(DataSource const & dataSource, std::shared_ptr<routing::CarModelFactory> carModelFactory);

  // Appends edges such as that edge.GetStartJunction() == junction to the |edges|.
  void GetOutgoingEdges(routing::Junction const & junction, EdgeVector & edges);
  // Appends edges such as that edge.GetEndJunction() == junction to the |edges|.
  void GetIngoingEdges(routing::Junction const & junction, EdgeVector & edges);

  // Appends edges such as that edge.GetStartJunction() == junction and edge.IsFake() == false
  // to the |edges|.
  void GetRegularOutgoingEdges(Junction const & junction, EdgeVector & edges);
  // Appends edges such as that edge.GetEndJunction() == junction and edge.IsFale() == false
  // to the |edges|.
  void GetRegularIngoingEdges(Junction const & junction, EdgeVector & edges);

  void FindClosestEdges(m2::PointD const & point, uint32_t const count,
                        std::vector<pair<Edge, Junction>> & vicinities) const;

  void AddFakeEdges(Junction const & junction,
                    std::vector<pair<Edge, Junction>> const & vicinities);

  void AddIngoingFakeEdge(Edge const & e);
  void AddOutgoingFakeEdge(Edge const & e);

  void ResetFakes() { m_graph.ResetFakes(); }

  void GetFeatureTypes(FeatureID const & featureId, feature::TypesHolder & types) const
  {
    m_graph.GetFeatureTypes(featureId, types);
  }

private:
  routing::FeaturesRoadGraph m_graph;
  std::map<Junction, EdgeVector> m_outgoingCache;
  std::map<Junction, EdgeVector> m_ingoingCache;
};

// @TODO Rename Score2
// @TODO Move these struct to a special file
using Score2 = uint32_t;

struct ScorePoint
{
  ScorePoint() = default;
  ScorePoint(Score2 score, m2::PointD const & point) : m_score(score), m_point(point) {}

  Score2 m_score = 0;
  m2::PointD m_point;
};

using ScorePointVec = std::vector<ScorePoint>;

struct ScoreEdge
{
  ScoreEdge(Score2 score, Graph::Edge const & edge) : m_score(score), m_edge(edge) {}

  Score2 m_score = 0;
  Graph::Edge m_edge;
};

using ScoreEdgeVec = std::vector<ScoreEdge>;

struct ScorePath
{
  ScorePath(Score2 score, Graph::EdgeVector && path) : m_score(score), m_path(move(path)) {}

  Score2 m_score = 0;
  Graph::EdgeVector m_path;
};

using ScorePathVec = std::vector<ScorePath>;
}  // namespace openlr
