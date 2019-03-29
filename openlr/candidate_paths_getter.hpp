#pragma once

#include "openlr/graph.hpp"
#include "openlr/openlr_model.hpp"
#include "openlr/road_info_getter.hpp"
#include "openlr/stats.hpp"

#include "geometry/point2d.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

namespace openlr
{
class CandidatePointsGetter;

class CandidatePathsGetter
{
public:
  CandidatePathsGetter(CandidatePointsGetter & pointsGetter, Graph & graph,
                       RoadInfoGetter & infoGetter, v2::Stats & stat)
    : m_pointsGetter(pointsGetter), m_graph(graph), m_infoGetter(infoGetter), m_stats(stat)
  {
  }

  bool GetLineCandidatesForPoints(std::vector<LocationReferencePoint> const & points,
                                  std::vector<ScorePathVec> & lineCandidates);

private:
  struct Link;
  using LinkPtr = std::shared_ptr<Link>;

  // @TODO Increece candidates number.
  size_t static constexpr kMaxCandidates = 5;
  size_t static constexpr kMaxFakeCandidates = 2;

  // TODO(mgsergio): Rename to Vertex.
  struct Link
  {
    Link(LinkPtr const & parent, Graph::Edge const & edge, double const distanceM,
         Score2 pointScore, Score2 rfcScore)
      : m_parent(parent)
      , m_edge(edge)
      , m_distanceM(distanceM)
      , m_pointScore(pointScore)
      , m_rfcScore(rfcScore)
      , m_hasFake((parent && parent->m_hasFake) || edge.IsFake())
    {
    }

    bool operator<(Link const & o) const;
    bool IsJunctionInPath(routing::Junction const & j) const;

    Graph::Edge GetStartEdge() const;

    LinkPtr const m_parent;
    Graph::Edge const m_edge;
    double const m_distanceM;
    Score2 m_pointScore;
    Score2 m_rfcScore;
    bool const m_hasFake;
  };

  struct CandidatePath
  {
    CandidatePath() = default;

    CandidatePath(LinkPtr const path, Score2 pointScore, Score2 rfcScore, Score2 bearingScore)
      : m_path(path)
//      , m_bearingDiff(bearingDiff)
//      , m_pathDistanceDiff(pathDistanceDiff)
//      , m_startPointDistance(startPointDistance)
      , m_pointScore(pointScore)
      , m_rfcScore(rfcScore)
      , m_bearingScore(bearingScore)
    {
    }

//    bool operator<(CandidatePath const & o) const { return GetPenalty() < o.GetPenalty(); }
    bool operator>(CandidatePath const & o) const { return GetScore() > o.GetScore(); }

    Score2 GetScore() const { return m_pointScore + m_rfcScore + m_bearingScore; }

//    double GetPenalty() const
//    {
//      return kBearingDiffFactor * m_bearingDiff + kPathDistanceFactor * m_pathDistanceDiff +
//             kPointDistanceFactor * m_startPointDistance;
//    }

    bool HasFakeEndings() const { return m_path && m_path->m_hasFake; }

    LinkPtr m_path = nullptr;
//    uint32_t m_bearingDiff = std::numeric_limits<uint32_t>::max();     // Domain is roughly [0, 30]
//    double m_pathDistanceDiff = std::numeric_limits<double>::max();    // Domain is roughly [0, 25]
//    double m_startPointDistance = std::numeric_limits<double>::max();  // Domain is roughly [0, 50]
    Score2 m_pointScore = 0;
    // @TODO Rename rfc
    Score2 m_rfcScore = 0;
    Score2 m_bearingScore = 0;
  };

  // Note: In all methods below if |isLastPoint| is true than algorithm should
  // calculate all parameters (such as bearing, distance to next point, etc.)
  // relative to the last point.
  // o ----> o ----> o <---- o.
  // 1       2       3       4
  //                         ^ isLastPoint = true.
  // To calculate bearing for points 1 to 3 one have to go beardist from
  // previous point to the next one (eg. from 1 to 2 and from 2 to 3).
  // For point 4 one have to go from 4 to 3 reversing directions. And
  // distance-to-next point is taken from point 3. You can learn more in
  // TomTom OpenLR spec.

  void GetStartLines(ScorePointVec const & points, bool const isLastPoint,
                     ScoreEdgeVec & edges);

  void GetAllSuitablePaths(ScoreEdgeVec const & startLines, bool isLastPoint,
                           double bearDistM, FunctionalRoadClass  functionalRoadClass,
                           FormOfWay formOfWay, std::vector<LinkPtr> & allPaths);

  void GetBestCandidatePaths(std::vector<LinkPtr> const & allPaths, bool const isLastPoint,
                             uint32_t const requiredBearing, double const bearDistM,
                             m2::PointD const & startPoint,
                             ScorePathVec & candidates);

  void GetLineCandidates(openlr::LocationReferencePoint const & p, bool const isLastPoint,
                         double const distanceToNextPointM,
                         ScoreEdgeVec const & edgeCandidates,
                         ScorePathVec & candidates);

  CandidatePointsGetter & m_pointsGetter;
  Graph & m_graph;
  RoadInfoGetter & m_infoGetter;
  v2::Stats & m_stats;
};
}  // namespace openlr
