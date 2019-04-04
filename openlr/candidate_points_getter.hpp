#pragma once

#include "openlr/graph.hpp"
#include "openlr/score_types.hpp"
#include "openlr/stats.hpp"

#include "indexer/data_source.hpp"

#include "geometry/point2d.hpp"

#include <cstddef>
#include <functional>
#include <vector>

namespace openlr
{
class CandidatePointsGetter
{
public:
  CandidatePointsGetter(size_t const maxJunctionCandidates, size_t const maxProjectionCandidates,
                        DataSource const & dataSource, Graph & graph)
    : m_maxJunctionCandidates(maxJunctionCandidates)
    , m_maxProjectionCandidates(maxProjectionCandidates)
    , m_dataSource(dataSource)
    , m_graph(graph)
  {
  }

  // @TODO Remove ScorePointVec & candidates
  void GetCandidatePointsEdges(m2::PointD const & p, bool isLastPoint, ScorePointVec & candidates, ScoreEdgeVec & edges)
  {
    GetJunctionPointCandidates(p, isLastPoint, candidates, edges);
    EnrichWithProjectionPoints(p, candidates, edges);
  }

private:
  void GetJunctionPointCandidates(m2::PointD const & p, bool isLastPoint,
                                  ScorePointVec & candidates, ScoreEdgeVec & candidateEdges);
  void EnrichWithProjectionPoints(m2::PointD const & p, ScorePointVec & candidates, ScoreEdgeVec & candidateEdges);
  bool IsJunction(m2::PointD const & p);
  openlr::Score2 GetScoreByDistance(m2::PointD const & point, m2::PointD const & candidate);

  size_t const m_maxJunctionCandidates;
  size_t const m_maxProjectionCandidates;

  DataSource const & m_dataSource;
  Graph & m_graph;
};
}  // namespace openlr
