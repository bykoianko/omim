#include "openlr/candidate_points_getter.hpp"

#include "openlr/helpers.hpp"

#include "routing/routing_helpers.hpp"
#include "routing/road_graph.hpp"

#include "storage/country_info_getter.hpp"

#include "geometry/mercator.hpp"

#include "base/assert.hpp"
#include "base/stl_helpers.hpp"

#include <set>
#include <utility>

using namespace routing;

namespace
{
// TODO(mgsergio): Get optimal value using experiments on a sample.
// Or start with small radius and scale it up when there are too few points.
double const kRadius = 30.0;
} //  namespace

namespace openlr
{
void CandidatePointsGetter::GetJunctionPointCandidates(m2::PointD const & p, bool isLastPoint,
                                                       ScorePointVec & candidates, ScoreEdgeVec & candidateEdges)
{
//  auto const rect = MercatorBounds::RectByCenterXYAndSizeInMeters(p, kRectSideMeters);
  auto const selectCandidates = [&](FeatureType & ft) {
//    if (!routing::IsCarRoad(feature::TypesHolder(ft)))
//      return;

    ft.ParseGeometry(FeatureType::BEST_GEOMETRY);
    ft.ForEachPoint(
        [&](m2::PointD const & candidate) {
//          if (rect.IsPointInside(candidate))
          if (MercatorBounds::DistanceOnEarth(p, candidate) < kRadius)
          {
//            LOG(LINFO, ("IsJunction(", MercatorBounds::ToLatLon(candidate), "):", IsJunction(candidate)));
            candidates.emplace_back(GetScoreByDistance(p, candidate), candidate);
          }
        },
        scales::GetUpperScale());
  };

  auto const rect = MercatorBounds::RectByCenterXYAndSizeInMeters(p, kRadius);
  m_dataSource.ForEachInRect(selectCandidates, rect, scales::GetUpperScale());

  // TODO: Move this to a separate stage.
  // 1030292476 Does not match. Some problem occur with points.
  // Either points duplicate or something alike. Check this
  // later. The idea to fix this was to move SortUnique to the stage
  // after enriching with projections.

  base::SortUnique(candidates,
                   [&](ScorePoint const & a, ScorePoint const & b) {
//                     return MercatorBounds::DistanceOnEarth(a.m_point, p) <
//                            MercatorBounds::DistanceOnEarth(b.m_point, p);
                     return a.m_score > b.m_score;
                   },
                   [](ScorePoint const & a, ScorePoint const & b) { return a.m_point == b.m_point; });

  candidates.resize(min(m_maxJunctionCandidates, candidates.size()));

  for (auto const & pc : candidates)
  {
    Graph::EdgeVector edges;
    if (!isLastPoint)
      m_graph.GetOutgoingEdges(Junction(pc.m_point, 0 /* altitude */), edges);
    else
      m_graph.GetIngoingEdges(Junction(pc.m_point, 0 /* altitude */), edges);

    for (auto const & e : edges)
      candidateEdges.emplace_back(pc.m_score, e);
  }
}

void CandidatePointsGetter::EnrichWithProjectionPoints(m2::PointD const & p,
                                                       ScorePointVec & candidates, ScoreEdgeVec & candidateEdges)
{
  m_graph.ResetFakes();

  vector<pair<Graph::Edge, Junction>> vicinities;
  m_graph.FindClosestEdges(p, static_cast<uint32_t>(m_maxProjectionCandidates), vicinities);
  for (auto const & v : vicinities)
  {
    auto const & edge = v.first;
    auto const & proj = v.second;

    CHECK(edge.HasRealPart(), ());
    CHECK(!edge.IsFake(), ());
//
//    if (PointsAreClose(edge.GetStartPoint(), proj.GetPoint()) ||
//        PointsAreClose(edge.GetEndPoint(), proj.GetPoint()))
//    {
//      continue;
//    }
//
//    auto const firstHalf = Edge::MakeFake(edge.GetStartJunction(), proj, edge);
//    auto const secondHalf = Edge::MakeFake(proj, edge.GetEndJunction(), edge);
//
//    m_graph.AddOutgoingFakeEdge(firstHalf);
//    m_graph.AddIngoingFakeEdge(firstHalf);
//    m_graph.AddOutgoingFakeEdge(secondHalf);
//    m_graph.AddIngoingFakeEdge(secondHalf);
    if (MercatorBounds::DistanceOnEarth(p, proj.GetPoint()) >= kRadius)
      continue;

    auto const scoreByDist = GetScoreByDistance(p, proj.GetPoint());
    candidates.emplace_back(scoreByDist, proj.GetPoint());
    candidateEdges.emplace_back(scoreByDist, edge);
  }
}

bool CandidatePointsGetter::IsJunction(m2::PointD const & p)
{
  Graph::EdgeVector outgoing;
  m_graph.GetRegularOutgoingEdges(routing::Junction(p, 0 /* altitude */), outgoing);

  Graph::EdgeVector ingoing;
  m_graph.GetRegularIngoingEdges(routing::Junction(p, 0 /* altitude */), ingoing);

  // @TODO segment may be in different mwms
  std::set<std::pair<uint32_t, uint32_t>> ids;
  for (auto const & e : outgoing)
    ids.insert(std::make_pair(e.GetFeatureId().m_index, e.GetSegId()));

  for (auto const & e : ingoing)
    ids.insert(std::make_pair(e.GetFeatureId().m_index, e.GetSegId()));

  return ids.size() >= 3;
}

openlr::Score2 CandidatePointsGetter::GetScoreByDistance(m2::PointD const & point,
                                                         m2::PointD const & candidate)
{
  // @TODO Add comment for this numbers
  openlr::Score2 constexpr kMaxScoreForDist = 70;
  double constexpr kMaxScoreDistM = 5.0;
  double const junctionFactor = IsJunction(candidate) ? 1.1 : 1.0;

  double const distM = MercatorBounds::DistanceOnEarth(point, candidate);
  double const score =
      (distM <= kMaxScoreDistM
           ? kMaxScoreForDist * junctionFactor
           : static_cast<double>(kMaxScoreForDist) * junctionFactor / (1 + distM - kMaxScoreDistM));
  CHECK_LESS_OR_EQUAL(score, static_cast<double>(kMaxScoreForDist) * junctionFactor, ());
  return static_cast<openlr::Score2>(score);
}
}  // namespace openlr
