#include "routing/joint_index.hpp"

namespace routing
{
Joint::Id JointIndex::InsertJoint(RoadPoint const & rp)
{
  Joint::Id const jointId = GetNumJoints();
  m_points.emplace_back(rp);
  m_offsets.emplace_back(m_points.size());
  return jointId;
}

pair<RoadPoint, RoadPoint> JointIndex::FindCommonFeature(Joint::Id jointId0,
                                                         Joint::Id jointId1) const
{
  for (size_t i = Begin(jointId0); i < End(jointId0); ++i)
  {
    RoadPoint const & rp0 = m_points[i];
    for (size_t j = Begin(jointId1); j < End(jointId1); ++j)
    {
      RoadPoint const & rp1 = m_points[j];
      if (rp0.GetFeatureId() == rp1.GetFeatureId())
        return make_pair(rp0, rp1);
    }
  }

  MYTHROW(RootException, ("Can't find common feature for joints", jointId0, jointId1));
}

void JointIndex::Build(RoadIndex const & roadIndex, uint32_t numJoints)
{
  // +2 is reserved space for start and finish.
  // + 1 is protection for 'End' method from out of bounds.
  // Call End(jointsAmount-1) requires more size, so add one more item.
  // Therefore m_offsets.size() == numJoints + 1,
  // And m_offsets.back() == m_points.size()
  m_offsets.reserve(numJoints + 1 + 2);
  m_offsets.assign(numJoints + 1, 0);

  // Calculate sizes.
  // Example: 2, 5, 3, 4, 2, 3, 0, where jointsAmount = 6
  roadIndex.ForEachRoad([this](uint32_t /* featureId */, RoadJointIds const & road) {
    road.ForEachJoint([this](uint32_t /* segId */, Joint::Id jointId) {
      ASSERT_LESS(jointId, m_offsets.size(), ());
      ++m_offsets[jointId];
    });
  });

  // Calculate shifted offsets.
  // Example: 0, 0, 2, 7, 10, 14, 16
  uint32_t sum = 0;
  uint32_t prevSum = 0;
  uint32_t prevPrevSum = 0;
  for (size_t i = 0; i < m_offsets.size(); ++i)
  {
    sum += m_offsets[i];
    m_offsets[i] = prevPrevSum;
    prevPrevSum = prevSum;
    prevSum = sum;
  }

  // +2 is reserved space for start and finish
  m_points.reserve(sum + 2);
  m_points.resize(sum);

  // Now fill points, m_offsets[nextId] is current incrementing begin.
  // Offsets after this operation: 0, 2, 7, 10, 14, 16, 19
  roadIndex.ForEachRoad([this](uint32_t featureId, RoadJointIds const & road) {
    road.ForEachJoint([this, featureId](uint32_t segId, Joint::Id jointId) {
      Joint::Id nextId = jointId + 1;
      ASSERT_LESS(nextId, m_offsets.size(), ());
      uint32_t & offset = m_offsets[nextId];
      m_points[offset] = {featureId, segId};
      ++offset;
    });
  });

  if (m_offsets[0] != 0)
    MYTHROW(RootException, ("Wrong offsets calculation: m_offsets[0] =", m_offsets[0]));

  if (m_offsets.back() != m_points.size())
    MYTHROW(RootException, ("Wrong offsets calculation: m_offsets.back() =", m_offsets.back(),
                            ", m_points.size()=", m_points.size()));
}
}  // namespace routing
