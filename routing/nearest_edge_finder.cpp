#include "routing/nearest_edge_finder.hpp"

#include "geometry/parametrized_segment.hpp"

#include "indexer/feature.hpp"

#include "base/assert.hpp"

namespace routing
{
using namespace std;

NearestEdgeFinder::NearestEdgeFinder(m2::PointD const & point)
    : m_point(point)
{
}

void NearestEdgeFinder::AddInformationSource(FeatureID const & featureId,
                                             IRoadGraph::JunctionVec const & junctions,
                                             bool bidirectional)
{
  if (!featureId.IsValid())
    return;

  Candidate res;

  size_t const count = junctions.size();
  ASSERT_GREATER(count, 1, ());
  for (size_t i = 1; i < count; ++i)
  {
    m2::ParametrizedSegment<m2::PointD> segment(junctions[i - 1].GetPoint(),
                                                junctions[i].GetPoint());

    m2::PointD const closestPoint = segment.ClosestPointTo(m_point);
    double const squaredDist = m_point.SquaredLength(closestPoint);

    if (squaredDist < res.m_squaredDist)
    {
      res.m_segId = static_cast<uint32_t>(i - 1);
      res.m_squaredDist = squaredDist;
    }
  }

  if (res.m_segId == Candidate::kInvalidSegmentId)
    return;

  // Closest point to |this->m_point| found. It has index |res.m_segId + 1| in |junctions|.
  size_t const idx = res.m_segId + 1;
  Junction const & segStart = junctions[idx - 1];
  Junction const & segEnd = junctions[idx];
  feature::TAltitude const startAlt = segStart.GetAltitude();
  feature::TAltitude const endAlt = segEnd.GetAltitude();
  m2::ParametrizedSegment<m2::PointD> segment(junctions[idx - 1].GetPoint(),
                                              junctions[idx].GetPoint());
  m2::PointD const closestPoint = segment.ClosestPointTo(m_point);

  double const segLenM = MercatorBounds::DistanceOnEarth(segStart.GetPoint(), segEnd.GetPoint());
  feature::TAltitude projPointAlt = feature::kDefaultAltitudeMeters;
  if (segLenM == 0.0)
  {
    projPointAlt = startAlt;
  }
  else
  {
    double const distFromStartM = MercatorBounds::DistanceOnEarth(segStart.GetPoint(), closestPoint);
    ASSERT_LESS_OR_EQUAL(distFromStartM, segLenM, (featureId));
    projPointAlt =
        startAlt + static_cast<feature::TAltitude>((endAlt - startAlt) * distFromStartM / segLenM);
  }

  res.m_fid = featureId;
  res.m_segStart = segStart;
  res.m_segEnd = segEnd;
  res.m_bidirectional = bidirectional;

  ASSERT_NOT_EQUAL(res.m_segStart.GetAltitude(), feature::kInvalidAltitude, ());
  ASSERT_NOT_EQUAL(res.m_segEnd.GetAltitude(), feature::kInvalidAltitude, ());
  res.m_projPoint = Junction(closestPoint, projPointAlt);

  m_candidates.push_back(res);
}

void NearestEdgeFinder::MakeResult(vector<pair<Edge, Junction>> & res, size_t const maxCountFeatures)
{
  sort(m_candidates.begin(), m_candidates.end(), [](Candidate const & r1, Candidate const & r2)
  {
    return (r1.m_squaredDist < r2.m_squaredDist);
  });

  res.clear();
  res.reserve(maxCountFeatures);
  
  for (Candidate const & candidate : m_candidates)
  {
    res.emplace_back(Edge::MakeReal(candidate.m_fid, true /* forward */, candidate.m_segId,
                                    candidate.m_segStart, candidate.m_segEnd),
                     candidate.m_projPoint);
    if (res.size() >= maxCountFeatures)
      return;

    if (candidate.m_bidirectional)
    {
      res.emplace_back(Edge::MakeReal(candidate.m_fid, false /* forward */, candidate.m_segId,
                                      candidate.m_segEnd, candidate.m_segStart),
                       candidate.m_projPoint);
      if (res.size() >= maxCountFeatures)
        return;
    }
  }
}
}  // namespace routing
