#include "index_graph.hpp"

#include "base/assert.hpp"
#include "base/exception.hpp"

namespace routing
{
IndexGraph::IndexGraph(unique_ptr<GeometryLoader> loader, shared_ptr<EdgeEstimator> estimator)
  : m_geometry(move(loader)), m_estimator(move(estimator))
{
  ASSERT(m_estimator, ());
}

void IndexGraph::GetOutgoingEdgesList(JointId jointId, vector<TEdgeType> & edges) const
{
  GetEdgesList(jointId, edges, true);
}

void IndexGraph::GetIngoingEdgesList(JointId jointId, vector<TEdgeType> & edges) const
{
  GetEdgesList(jointId, edges, false);
}

double IndexGraph::HeuristicCostEstimate(JointId jointFrom, JointId jointTo) const
{
  return m_estimator->CalcHeuristic(GetPoint(jointFrom), GetPoint(jointTo));
}

m2::PointD const & IndexGraph::GetPoint(JointId jointId) const
{
  return m_geometry.GetPoint(m_jointIndex.GetFSeg(jointId));
}

void IndexGraph::Export(vector<Joint> const & joints)
{
  m_fsegIndex.Import(joints);
  m_jointIndex.Build(m_fsegIndex, joints.size());
}

JointId IndexGraph::InsertJoint(FSegId const & fseg)
{
  JointId const existId = m_fsegIndex.GetJointId(fseg);
  if (existId != kInvalidJointId)
    return existId;

  JointId const jointId = m_jointIndex.InsertJoint(fseg);
  m_fsegIndex.AddJoint(fseg, jointId);
  return jointId;
}

vector<FSegId> IndexGraph::RedressRoute(vector<JointId> const & route) const
{
  vector<FSegId> fsegs;
  if (route.size() < 2)
  {
    if (route.size() == 1)
      fsegs.emplace_back(m_jointIndex.GetFSeg(route[0]));
    return fsegs;
  }

  fsegs.reserve(route.size() * 2);

  for (size_t i = 0; i < route.size() - 1; ++i)
  {
    JointId const prevJoint = route[i];
    JointId const nextJoint = route[i + 1];

    auto const & pair = m_jointIndex.FindCommonFeature(prevJoint, nextJoint);
    if (i == 0)
      fsegs.push_back(pair.first);

    uint32_t const featureId = pair.first.GetFeatureId();
    uint32_t const segFrom = pair.first.GetSegId();
    uint32_t const segTo = pair.second.GetSegId();

    if (segFrom < segTo)
    {
      for (uint32_t seg = segFrom + 1; seg < segTo; ++seg)
        fsegs.push_back({featureId, seg});
    }
    else if (segFrom > segTo)
    {
      for (uint32_t seg = segFrom - 1; seg > segTo; --seg)
        fsegs.push_back({featureId, seg});
    }
    else
      MYTHROW(RootException,
              ("Wrong equality segFrom = segTo =", segFrom, ", featureId = ", featureId));

    fsegs.push_back(pair.second);
  }

  return fsegs;
}

inline void IndexGraph::AddNeigborEdge(RoadGeometry const & road, vector<TEdgeType> & edges,
                                       FSegId fseg, bool forward) const
{
  pair<JointId, uint32_t> const & pair = m_fsegIndex.FindNeigbor(fseg, forward);
  if (pair.first != kInvalidJointId)
  {
    double const distance = m_estimator->CalcEdgesWeight(road, fseg.GetSegId(), pair.second);
    edges.push_back({pair.first, distance});
  }
}

inline void IndexGraph::GetEdgesList(JointId jointId, vector<TEdgeType> & edges, bool forward) const
{
  edges.clear();

  m_jointIndex.ForEachFtSeg(jointId, [this, &edges, forward](FSegId const & fseg) {
    RoadGeometry const & road = m_geometry.GetRoad(fseg.GetFeatureId());
    if (!road.IsRoad())
      return;

    bool const twoWay = !road.IsOneWay();
    if (!forward || twoWay)
      AddNeigborEdge(road, edges, fseg, false);

    if (forward || twoWay)
      AddNeigborEdge(road, edges, fseg, true);
  });
}
}  // namespace routing
