#include "openlr/candidate_paths_getter.hpp"

#include "openlr/candidate_points_getter.hpp"
#include "openlr/graph.hpp"
#include "openlr/helpers.hpp"
#include "openlr/openlr_model.hpp"

#include "routing/road_graph.hpp"

#include "platform/location.hpp"

#include "geometry/angles.hpp"

#include "base/stl_helpers.hpp"

#include <algorithm>
#include <iterator>
#include <functional>
#include <limits>
#include <queue>
#include <set>
#include <tuple>

using namespace std;
using namespace routing;

namespace openlr
{
namespace
{
int const kNumBuckets = 256;
double const kAnglesInBucket = 360.0 / kNumBuckets;

m2::PointD PointAtSegmentM(m2::PointD const & p1, m2::PointD const & p2, double const distanceM)
{
  auto const v = p2 - p1;
  auto const l = v.Length();
  auto const L = MercatorBounds::DistanceOnEarth(p1, p2);
  auto const delta = distanceM * l / L;
  return PointAtSegment(p1, p2, delta);
}

double ToAngleInDeg(uint32_t angleInBuckets)
{
  CHECK_GREATER_OR_EQUAL(angleInBuckets, 0, ());
  CHECK_LESS_OR_EQUAL(angleInBuckets, 255, ());
  return base::clamp(kAnglesInBucket * static_cast<double>(angleInBuckets), 0.0, 360.0);
}

uint32_t BearingInDeg(m2::PointD const & a, m2::PointD const & b)
{
  auto const angle = location::AngleToBearing(base::RadToDeg(ang::AngleTo(a, b)));
  CHECK_LESS_OR_EQUAL(angle, 360.0, ("Angle should be less than or equal to 360."));
  CHECK_GREATER_OR_EQUAL(angle, 0.0, ("Angle should be greater than or equal to 0."));
//  return base::clamp(angle / kAnglesInBucket, 0.0, 255.0);
  return angle;
}

double DifferenceInDeg(double a1, double a2)
{
  auto const diff = 180.0 - abs(abs(a1 - a2) - 180.0);
  CHECK_LESS_OR_EQUAL(diff, 180.0, ("Difference should be less than or equal to 360."));
  CHECK_GREATER_OR_EQUAL(diff, 0.0, ("Difference should be greater than or equal to 0."));
  return diff;
}

// This class is used to get correct points for further bearing calculations.
// Depending on |isLastPoint| it either calculates those points straightforwardly
// or reverses directions and then calculates.
class BearingPointsSelector
{
public:
  BearingPointsSelector(uint32_t const bearDistM, bool const isLastPoint)
    : m_bearDistM(bearDistM), m_isLastPoint(isLastPoint)
  {
  }

  m2::PointD GetBearingStartPoint(Graph::Edge const & e) const
  {
    return m_isLastPoint ? e.GetEndPoint() : e.GetStartPoint();
  }

  m2::PointD GetBearingEndPoint(Graph::Edge const & e, double const distanceM)
  {
    if (distanceM < m_bearDistM && m_bearDistM <= distanceM + EdgeLength(e))
    {
      auto const edgeLen = EdgeLength(e);
      auto const edgeBearDist = min(m_bearDistM - distanceM, edgeLen);
      ASSERT_LESS_OR_EQUAL(edgeBearDist, edgeLen, ());
      return m_isLastPoint ? PointAtSegmentM(e.GetEndPoint(), e.GetStartPoint(),
                                             static_cast<double>(edgeBearDist))
                           : PointAtSegmentM(e.GetStartPoint(), e.GetEndPoint(),
                                             static_cast<double>(edgeBearDist));
    }
    return m_isLastPoint ? e.GetStartPoint() : e.GetEndPoint();
  }

private:
  double m_bearDistM;
  bool m_isLastPoint;
};
}  // namespace

// CandidatePathsGetter::Link ----------------------------------------------------------------------
bool CandidatePathsGetter::Link::operator<(Link const & o) const
{
  if (m_distanceM != o.m_distanceM)
    return m_distanceM < o.m_distanceM;

  if (m_edge != o.m_edge)
    return m_edge < o.m_edge;

  if (m_parent == o.m_parent)
    return false;

  if (m_parent && o.m_parent)
    return *m_parent < *o.m_parent;

  if (!m_parent)
    return true;

  return false;
}

Graph::Edge CandidatePathsGetter::Link::GetStartEdge() const
{
  auto * start = this;
  while (start->m_parent)
    start = start->m_parent.get();

  return start->m_edge;
}

bool CandidatePathsGetter::Link::IsJunctionInPath(routing::Junction const & j) const
{
  for (auto * l = this; l; l = l->m_parent.get())
  {
    if (l->m_edge.GetEndJunction() == j)
    {
      LOG(LDEBUG, ("A loop detected, skipping..."));
      return true;
    }
  }

  return false;
}

// CandidatePathsGetter ----------------------------------------------------------------------------
bool CandidatePathsGetter::GetLineCandidatesForPoints(
    vector<LocationReferencePoint> const & points,
    vector<ScorePathVec> & lineCandidates)
{
  for (size_t i = 0; i < points.size(); ++i)
  {
    if (i != points.size() - 1 && points[i].m_distanceToNextPoint == 0)
    {
      LOG(LINFO, ("Distance to next point is zero. Skipping the whole segment"));
      ++m_stats.m_dnpIsZero;
      return false;
    }

    lineCandidates.emplace_back();
    auto const isLastPoint = i == points.size() - 1;
    double const distanceToNextPointM =
        (isLastPoint ? points[i - 1] : points[i]).m_distanceToNextPoint;

    ScorePointVec pointCandidates;
    ScoreEdgeVec edgesCandidates;
    m_pointsGetter.GetCandidatePointsEdges(MercatorBounds::FromLatLon(points[i].m_latLon), isLastPoint,
                                           pointCandidates, edgesCandidates);
//    LOG(LINFO, ("edgesCandidates size:", edgesCandidates.size()));
    GetLineCandidates(points[i], isLastPoint, distanceToNextPointM, edgesCandidates,
                      lineCandidates.back());

    if (lineCandidates.back().empty())
    {
      LOG(LINFO, ("No candidate lines found for point", points[i].m_latLon, "Giving up"));
      ++m_stats.m_noCandidateFound;
      return false;
    }
  }

  ASSERT_EQUAL(lineCandidates.size(), points.size(), ());

  return true;
}

// @TODO Remove this method
void CandidatePathsGetter::GetStartLines(ScorePointVec const & points, bool const isLastPoint,
                                         ScoreEdgeVec & scoreEdges)
{
  for (auto const & pc : points)
  {
    Graph::EdgeVector edges;
    if (!isLastPoint)
      m_graph.GetOutgoingEdges(Junction(pc.m_point, 0 /* altitude */), edges);
    else
      m_graph.GetIngoingEdges(Junction(pc.m_point, 0 /* altitude */), edges);

    for (auto const & e : edges)
      scoreEdges.emplace_back(pc.m_score, e);
  }

  // Same edges may start on different points if those points are close enough.
//  base::SortUnique(scoreEdges, base::LessBy(&ScoreEdge::m_score),
//                   [](ScoreEdge const & e1, ScoreEdge const & e2) {
//                     return EdgesAreAlmostEqual(e1.m_edge, e2.m_edge);
//                   });
}

void CandidatePathsGetter::GetAllSuitablePaths(ScoreEdgeVec const & startLines,
                                               bool isLastPoint, double bearDistM,
                                               FunctionalRoadClass functionalRoadClass,
                                               FormOfWay formOfWay, vector<LinkPtr> & allPaths)
{
  queue<LinkPtr> q;

  for (auto const & e : startLines)
  {
    Score2 rfcScore = 0; // road functional class score
    if (!PassesRestriction(e.m_edge, functionalRoadClass, formOfWay, m_infoGetter, rfcScore))
      continue;

    auto const u =
        make_shared<Link>(nullptr /* parent */, e.m_edge, 0 /* distanceM */, e.m_score, rfcScore);
    q.push(u);
  }

  while (!q.empty())
  {
    auto const u = q.front();
    q.pop();

    auto const & currentEdge = u->m_edge;
    auto const currentEdgeLen = EdgeLength(currentEdge);

    // TODO(mgsergio): Maybe weak this constraint a bit.
    if (u->m_distanceM + currentEdgeLen >= bearDistM)
    {
      allPaths.push_back(u);
      continue;
    }

    ASSERT_LESS(u->m_distanceM + currentEdgeLen, bearDistM, ());

    Graph::EdgeVector edges;
    if (!isLastPoint)
      m_graph.GetOutgoingEdges(currentEdge.GetEndJunction(), edges);
    else
      m_graph.GetIngoingEdges(currentEdge.GetStartJunction(), edges);

    for (auto const & e : edges)
    {
      // Fake edges are allowed only at the start/end of the path.
      CHECK(!e.IsFake(), ());
//      if (e.IsFake())
//        continue;

      if (EdgesAreAlmostEqual(e.GetReverseEdge(), currentEdge))
        continue;

      ASSERT(currentEdge.HasRealPart(), ());

      Score2 frcScore = 0;
      if (!PassesRestriction(e, functionalRoadClass, formOfWay, m_infoGetter, frcScore))
        continue;

      if (u->IsJunctionInPath(e.GetEndJunction()))
        continue;

      // Functional road class for a path is minimum value of frc of its edges.
      auto const p = make_shared<Link>(u, e, u->m_distanceM + currentEdgeLen, u->m_pointScore,
                                       std::min(frcScore, u->m_rfcScore));
      q.push(p);
    }
  }
}

void CandidatePathsGetter::GetBestCandidatePaths(
    vector<LinkPtr> const & allPaths, bool const isLastPoint, uint32_t const requiredBearing,
    double const bearDistM, m2::PointD const & startPoint, ScorePathVec & candidates)
{
  CHECK_GREATER_OR_EQUAL(requiredBearing, 0, ());
  CHECK_LESS_OR_EQUAL(requiredBearing, 255, ());
//  LOG(LINFO, ("requiredBearing:", requiredBearing));
//  double constexpr kBearingDiffFactor = 5;
//  double constexpr kPathDistanceFactor = 1;
//  double constexpr kPointDistanceFactor = 2;

// @TODO Use multiset
  set<CandidatePath, std::greater<CandidatePath>> candidatePaths;
//  set<CandidatePath, std::greater<CandidatePath>> fakeEndingsCandidatePaths;

  BearingPointsSelector pointsSelector(bearDistM, isLastPoint);
  for (auto const & l : allPaths)
  {
    auto const bearStartPoint = pointsSelector.GetBearingStartPoint(l->GetStartEdge());
//    auto const startPointsDistance = MercatorBounds::DistanceOnEarth(bearStartPoint, startPoint);

    // Number of edges counting from the last one to check bearing on. According to OpenLR spec
    // we have to check bearing on a point that is no longer than 25 meters traveling down the path.
    // But sometimes we may skip the best place to stop and generate a candidate. So we check several
    // edges before the last one to avoid such a situation. Number of iterations is taken
    // by intuition.
    // Example:
    // o -------- o  { Partners segment. }
    // o ------- o --- o { Our candidate. }
    //               ^ 25m
    //           ^ This one may be better than
    //                 ^ this one.
    // So we want to check them all.
    uint32_t traceBackIterationsLeft = 3;
    for (auto part = l; part; part = part->m_parent)
    {
      CHECK(!part->m_hasFake, ());

      if (traceBackIterationsLeft == 0)
        break;

      --traceBackIterationsLeft;

      auto const bearEndPoint =
          pointsSelector.GetBearingEndPoint(part->m_edge, part->m_distanceM);

      auto const bearingDeg = BearingInDeg(bearStartPoint, bearEndPoint);
      double const requiredBearingDeg = ToAngleInDeg(requiredBearing);
      // @TODO Do it using some base code
      double const diff = DifferenceInDeg(bearingDeg, requiredBearingDeg);

//      auto const bearingDiff = AbsDifference(bearing, requiredBearing);

      if (diff > 50.0)
        continue;

//      auto const pathDistDiff = AbsDifference(part->m_distanceM, bearDistM);
//      double const bearingScore =
//          kBearingDiffFactor * bearingDiff + kPathDistanceFactor * pathDistDiff +
//          kPointDistanceFactor * startPointDistance;
      auto const bearingScore = static_cast<Score2>(60.0 * 1.0 / (1.0 + diff / 4.0));


      // @TODO Remove fake edges.
//      if (part->m_hasFake)
//      {
//        fakeEndingsCandidatePaths.emplace(part, bearingDiff, pathDistDiff, startPointsDistance,
//                                          part->m_pointScore, part->m_rfcScore, bearingScore);
//        fakeEndingsCandidatePaths.emplace(part, part->m_pointScore / 3.0, part->m_rfcScore, bearingScore);
//      }
//      else
//      {
//        candidatePaths.emplace(part, bearingDiff, pathDistDiff, startPointsDistance,
//                               part->m_pointScore, part->m_rfcScore, bearingScore);
        candidatePaths.emplace(part, part->m_pointScore, part->m_rfcScore, bearingScore);
//      }
    }
  }

  CHECK(
      none_of(begin(candidatePaths), end(candidatePaths), mem_fn(&CandidatePath::HasFakeEndings)),
      ());
//  ASSERT(fakeEndingsCandidatePaths.empty() ||
//             any_of(begin(fakeEndingsCandidatePaths), end(fakeEndingsCandidatePaths),
//                    mem_fn(&CandidatePath::HasFakeEndings)),
//         ());

//  LOG(LINFO, ("===Score==="));
//  for (auto const & p : candidatePaths)
//    LOG(LINFO, ("Point score:", p.m_pointScore, ", rfc score:", p.m_rfcScore, "bearing score:", p.m_bearingScore));

  vector<CandidatePath> paths;
  copy_n(begin(candidatePaths), min(static_cast<size_t>(kMaxCandidates), candidatePaths.size()),
         back_inserter(paths));

//  copy_n(begin(fakeEndingsCandidatePaths),
//         min(static_cast<size_t>(kMaxFakeCandidates), fakeEndingsCandidatePaths.size()),
//         back_inserter(paths));

//  LOG(LINFO, ("List candidate paths..."));
  for (auto const & path : paths)
  {
//    LOG(LDEBUG, ("CP:", path.m_bearingDiff, path.m_pathDistanceDiff, path.m_startPointDistance));
//    LOG(LINFO, ("pointScore:", path.m_pointScore, "rfcScore", path.m_rfcScore, "bearingScore",
//        path.m_bearingScore));
    Graph::EdgeVector edges;
    for (auto * p = path.m_path.get(); p; p = p->m_parent.get())
      edges.push_back(p->m_edge);
    if (!isLastPoint)
      reverse(begin(edges), end(edges));

    // @TODO Fills score
    candidates.emplace_back(path.GetScore(), move(edges));
  }
}

void CandidatePathsGetter::GetLineCandidates(openlr::LocationReferencePoint const & p,
                                             bool const isLastPoint,
                                             double const distanceToNextPointM,
                                             ScoreEdgeVec const & edgeCandidates,
                                             ScorePathVec & candidates)
{
  double const kDefaultBearDistM = 25.0;
  double const bearDistM = min(kDefaultBearDistM, distanceToNextPointM);

  LOG(LINFO, ("BearDist is", bearDistM));

  ScoreEdgeVec const & startLines = edgeCandidates;
//  GetStartLines(pointCandidates, isLastPoint, startLines);

  LOG(LINFO, (startLines.size(), "start line candidates found for point (LatLon)", p.m_latLon));
  LOG(LDEBUG, ("Listing start lines:"));
  for (auto const & e : startLines)
    LOG(LDEBUG, (LogAs2GisPath(e.m_edge)));

  auto const startPoint = MercatorBounds::FromLatLon(p.m_latLon);

  vector<LinkPtr> allPaths;
  GetAllSuitablePaths(startLines, isLastPoint, bearDistM, p.m_functionalRoadClass, p.m_formOfWay,
                      allPaths);
//  for (auto const & p : allPaths)
//    LOG(LINFO, ("For", p->m_edge, "pointScore:", p->m_pointScore, "rfcScore:", p->m_rfcScore));

  GetBestCandidatePaths(allPaths, isLastPoint, p.m_bearing, bearDistM, startPoint, candidates);
  // Sorting by increasing order.
  sort(candidates.begin(), candidates.end(),
       [](ScorePath const & s1, ScorePath const & s2) { return s1.m_score > s2.m_score; });
//  for (auto const & c : candidates)
//    LOG(LINFO, ("Score:", c.m_score));
  LOG(LDEBUG, (candidates.size(), "candidate paths found for point (LatLon)", p.m_latLon));
}
}  // namespace openlr
