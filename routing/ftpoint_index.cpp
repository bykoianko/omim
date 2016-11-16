#include "routing/ftpoint_index.hpp"

#include "base/exception.hpp"

#include "std/utility.hpp"

namespace routing
{
void FtPointIndex::Import(vector<Joint> const & joints)
{
  for (Joint::Id jointId = 0; jointId < joints.size(); ++jointId)
  {
    Joint const & joint = joints[jointId];
    for (uint32_t i = 0; i < joint.GetSize(); ++i)
    {
      FtPoint const & entry = joint.GetEntry(i);
      RoadJointIds & roadJoints = m_roads[entry.GetFeatureId()];
      roadJoints.AddJoint(entry.GetPointId(), jointId);
    }
  }
}

pair<Joint::Id, uint32_t> FtPointIndex::FindNeighbor(FtPoint ftp, bool forward) const
{
  auto const it = m_roads.find(ftp.GetFeatureId());
  if (it == m_roads.cend())
    MYTHROW(RootException, ("FtPointIndex doesn't contains feature", ftp.GetFeatureId()));

  RoadJointIds const & joints = it->second;
  int32_t const step = forward ? 1 : -1;

  for (uint32_t segId = ftp.GetPointId() + step; segId < joints.GetSize(); segId += step)
  {
    Joint::Id const jointId = joints.GetJointId(segId);
    if (jointId != Joint::kInvalidId)
      return make_pair(jointId, segId);
  }

  return make_pair(Joint::kInvalidId, 0);
}
}  // namespace routing
