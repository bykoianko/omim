#include "routing/joint_index.hpp"

namespace routing
{
Joint::Id JointIndex::InsertJoint(FSegId const & fseg)
{
  Joint::Id const jointId = m_slices.size();
  m_slices.emplace_back(Slice(m_fsegs.size(), m_fsegs.size() + 1));
  m_fsegs.emplace_back(fseg);
  return jointId;
}

pair<FSegId, FSegId> JointIndex::FindCommonFeature(Joint::Id jointId0, Joint::Id jointId1) const
{
  Slice const & slice0 = GetSlice(jointId0);
  Slice const & slice1 = GetSlice(jointId1);

  for (size_t i = slice0.Begin(); i < slice0.End(); ++i)
  {
    FSegId const & fseg0 = m_fsegs[i];
    for (size_t j = slice1.Begin(); j < slice1.End(); ++j)
    {
      FSegId const & fseg1 = m_fsegs[j];
      if (fseg0.GetFeatureId() == fseg1.GetFeatureId())
        return make_pair(fseg0, fseg1);
    }
  }

  MYTHROW(RootException, ("Can't find common feature for joints", jointId0, jointId1));
}

void JointIndex::Build(FSegIndex const & fsegIndex, uint32_t jointsAmount)
{
  // +2 is reserved space for start and finish
  m_slices.reserve(jointsAmount + 2);
  m_slices.assign(jointsAmount, {0, 0});

  fsegIndex.ForEachRoad([this](uint32_t /* featureId */, RoadJointIds const & road) {
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
  m_fsegs.reserve(offset + 2);
  m_fsegs.resize(offset);

  fsegIndex.ForEachRoad([this](uint32_t featureId, RoadJointIds const & road) {
    road.ForEachJoint([this, featureId](uint32_t segId, Joint::Id jointId) {
      ASSERT_LESS(jointId, m_slices.size(), ());
      Slice & slice = m_slices[jointId];
      m_fsegs[slice.End()] = {featureId, segId};
      slice.IncSize();
    });
  });
}
}  // namespace routing
