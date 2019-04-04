#pragma once

#include "geometry/point2d.hpp"

#include <cstdint>
#include <vector>

namespace openlr
{
// @TODO Rename Score2
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
