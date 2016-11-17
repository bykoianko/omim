#include "routing/road_index.hpp"

#include "base/exception.hpp"

#include "std/utility.hpp"

namespace routing
{
void RoadIndex::Import(vector<Joint> const & joints)
{
  for (Joint::Id jointId = 0; jointId < joints.size(); ++jointId)
  {
    Joint const & joint = joints[jointId];
    for (uint32_t i = 0; i < joint.GetSize(); ++i)
    {
      RoadPoint const & entry = joint.GetEntry(i);
      RoadJointIds & roadJoints = m_roads[entry.GetFeatureId()];
      roadJoints.AddJoint(entry.GetPointId(), jointId);
    }
  }
}

pair<Joint::Id, uint32_t> RoadIndex::FindNeighbor(RoadPoint rp, bool forward) const
{
  auto const it = m_roads.find(rp.GetFeatureId());
  if (it == m_roads.cend())
    MYTHROW(RootException, ("FtPointIndex doesn't contains feature", rp.GetFeatureId()));

  RoadJointIds const & joints = it->second;
  int32_t const step = forward ? 1 : -1;

  for (uint32_t segId = rp.GetPointId() + step; segId < joints.GetSize(); segId += step)
  {
    Joint::Id const jointId = joints.GetJointId(segId);
    if (jointId != Joint::kInvalidId)
      return make_pair(jointId, segId);
  }

  return make_pair(Joint::kInvalidId, 0);
}
}  // namespace routing
