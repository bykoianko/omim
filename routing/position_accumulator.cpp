#include "routing/position_accumulator.hpp"

#include "geometry/mercator.hpp"

#include "base/assert.hpp"

#include <algorithm>

double constexpr PositionAccumulator::kMinTrackLengthM;
double constexpr PositionAccumulator::kMinGoodSegmentLengthM;
double constexpr PositionAccumulator::kMaxGoodSegmentLengthM;
double constexpr PositionAccumulator::kMinValidSegmentLengthM;
double constexpr PositionAccumulator::kMaxValidSegmentLengthM;

void PositionAccumulator::PushNextPoint(m2::PointD const & point)
{
  double const lenM =
      m_points.empty() ? 0.0 : MercatorBounds::DistanceOnEarth(point, m_points.back());

  // Only the last segment is used if it has a good length.
  if (lenM >= kMinGoodSegmentLengthM && lenM <= kMaxGoodSegmentLengthM)
  {
    m_points.erase(m_points.begin(), m_points.end() - 1);
    m_trackLengthM = lenM;
    m_points.push_back(point);
    CHECK_EQUAL(m_points.size(), 2, ());
    return;
  }

  // If the last segment is too long it tells nothing about an end user direction.
  // And the history is not actual.
  if (lenM > kMaxValidSegmentLengthM)
  {
    Clear();
    return;
  }

  // If the last segment is too short it means an end user stays we there's no information
  // about it's direction. If m_points.empty() == true it means |point| is the first point.
  if (!m_points.empty() && lenM < kMinValidSegmentLengthM)
    return;

  // If |m_points| is empty |point| should be added any way.
  // If the size of |m_points| is 1 and |lenM| is valid |point| should be added.
  if (m_points.size() < 2)
  {
    CHECK_EQUAL(m_trackLengthM, 0.0, ());
    m_trackLengthM = lenM;
    m_points.push_back(point);
    return;
  }

  // If after adding |point| to |m_points| and removing the farthest point the segment length
  // is less than |kMinTrackLengthM| we just adding |point|.
  double farthestSegmentLenM = MercatorBounds::DistanceOnEarth(m_points[1], m_points[0]);
  if (m_trackLengthM + lenM - farthestSegmentLenM <= kMinTrackLengthM)
  {
    m_trackLengthM += lenM;
    m_points.push_back(point);
    return;
  }

  // Removing the farthest point if length of the track |m_points[1]|, ..., |m_points.back()|, |point|
  // is more than |kMinTrackLengthM|.
  while (m_trackLengthM + lenM - farthestSegmentLenM > kMinTrackLengthM && m_points.size() > 2)
  {
    m_trackLengthM -= farthestSegmentLenM;
    m_points.pop_front();
    farthestSegmentLenM = MercatorBounds::DistanceOnEarth(m_points[1], m_points[0]);
  }
  m_trackLengthM += lenM;
  m_points.push_back(point);
  // @TODO(bykoianko) Error in |m_trackLengthM| may be accumulated.
}

void PositionAccumulator::PositionAccumulator::Clear()
{
  m_points.clear();
  m_trackLengthM = 0.0;
}

m2::PointD PositionAccumulator::PositionAccumulator::GetDirection() const
{
  if (m_points.size() <= 1)
    return m2::PointD::Zero();

  return m_points.back() - m_points.front();
}
