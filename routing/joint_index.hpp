#pragma once

#include "routing/joint.hpp"
#include "routing/road_index.hpp"
#include "routing/road_point.hpp"

#include "std/vector.hpp"

namespace routing
{
// JointIndex contains mapping from Joint::Id to FtSegs.
//
// It is vector<Joint> conceptually.
// Technically Joint entries are joined into the single vector to reduce allocations overheads.
class JointIndex final
{
public:
  // Read comments in Build method about -1.
  size_t GetNumJoints() const { return m_offsets.size() - 1; }
  size_t GetNumPoints() const { return m_points.size(); }
  RoadPoint GetFtPoint(Joint::Id jointId) const { return m_points[Begin(jointId)]; }

  template <typename F>
  void ForEachPoint(Joint::Id jointId, F && f) const
  {
    for (uint32_t i = Begin(jointId); i < End(jointId); ++i)
      f(m_points[i]);
  }

  void Build(RoadIndex const & roadIndex, uint32_t numJoints);
  pair<RoadPoint, RoadPoint> FindCommonFeature(Joint::Id jointId0, Joint::Id jointId1) const;
  Joint::Id InsertJoint(RoadPoint const & rp);

private:
  // Begin index for jointId entries.
  uint32_t Begin(Joint::Id jointId) const
  {
    ASSERT_LESS(jointId, m_offsets.size(), ());
    return m_offsets[jointId];
  }

  // End index (not inclusive) for jointId entries.
  uint32_t End(Joint::Id jointId) const
  {
    Joint::Id const nextId = jointId + 1;
    ASSERT_LESS(nextId, m_offsets.size(), ());
    return m_offsets[nextId];
  }

  vector<uint32_t> m_offsets;
  vector<RoadPoint> m_points;
};
}  // namespace routing
