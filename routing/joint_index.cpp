#include "routing/joint_index.hpp"

namespace routing
{
Joint::Id JointIndex::InsertJoint(RoadPoint const & rp)
{
  Joint::Id const jointId = m_slices.size();
  m_slices.emplace_back(Slice(m_points.size(), m_points.size() + 1));
  m_points.emplace_back(rp);
  return jointId;
}

pair<RoadPoint, RoadPoint> JointIndex::FindCommonFeature(Joint::Id jointId0,
                                                         Joint::Id jointId1) const
{
  Slice const & slice0 = GetSlice(jointId0);
  Slice const & slice1 = GetSlice(jointId1);

  for (size_t i = slice0.Begin(); i < slice0.End(); ++i)
  {
    RoadPoint const & rp0 = m_points[i];
    for (size_t j = slice1.Begin(); j < slice1.End(); ++j)
    {
      RoadPoint const & rp1 = m_points[j];
      if (rp0.GetFeatureId() == rp1.GetFeatureId())
        return make_pair(rp0, rp1);
    }
  }

  MYTHROW(RootException, ("Can't find common feature for joints", jointId0, jointId1));
}

void JointIndex::Build(RoadIndex const & ftPointIndex, uint32_t jointsAmount)
{
  // +2 is reserved space for start and finish
  m_slices.reserve(jointsAmount + 2);
  m_slices.assign(jointsAmount, {0, 0});

  ftPointIndex.ForEachRoad([this](uint32_t /* featureId */, RoadJointIds const & road) {
    road.ForEachJoint([this](uint32_t /* segId */, Joint::Id jointId) {
      ASSERT_LESS(jointId, m_slices.size(), ());
      m_slices[jointId].IncSize();
    });
  });

  uint32_t offset = 0;
  for (size_t i = 0; i < m_slices.size(); ++i)
  {
    Slice & slice = m_slices[i];
    uint32_t const size = slice.End();
    ASSERT_GREATER(size, 0, ());

    slice.Assign(offset);
    offset += size;
  }

  // +2 is reserved space for start and finish
  m_points.reserve(offset + 2);
  m_points.resize(offset);

  ftPointIndex.ForEachRoad([this](uint32_t featureId, RoadJointIds const & road) {
    road.ForEachJoint([this, featureId](uint32_t segId, Joint::Id jointId) {
      ASSERT_LESS(jointId, m_slices.size(), ());
      Slice & slice = m_slices[jointId];
      m_points[slice.End()] = {featureId, segId};
      slice.IncSize();
    });
  });
}
}  // namespace routing
