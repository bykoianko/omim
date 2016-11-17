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

void IndexGraph::GetOutgoingEdgesList(Joint::Id jointId, vector<TEdgeType> & edges) const
{
  GetEdgesList(jointId, true, edges);
}

void IndexGraph::GetIngoingEdgesList(Joint::Id jointId, vector<TEdgeType> & edges) const
{
  GetEdgesList(jointId, false, edges);
}

double IndexGraph::HeuristicCostEstimate(Joint::Id jointFrom, Joint::Id jointTo) const
{
  return m_estimator->CalcHeuristic(GetPoint(jointFrom), GetPoint(jointTo));
}

m2::PointD const & IndexGraph::GetPoint(Joint::Id jointId) const
{
  return m_geometry.GetPoint(m_jointIndex.GetFtPoint(jointId));
}

void IndexGraph::Import(vector<Joint> const & joints)
{
  m_roadIndex.Import(joints);
  m_jointIndex.Build(m_roadIndex, joints.size());
}

Joint::Id IndexGraph::InsertJoint(RoadPoint const & rp)
{
  Joint::Id const existId = m_roadIndex.GetJointId(rp);
  if (existId != Joint::kInvalidId)
    return existId;

  Joint::Id const jointId = m_jointIndex.InsertJoint(rp);
  m_roadIndex.AddJoint(rp, jointId);
  return jointId;
}

// Add intermediate points to route (those doesn't correspond any joint).
//
// Also convert joint id to feature id, point id.
vector<RoadPoint> IndexGraph::RedressRoute(vector<Joint::Id> const & route) const
{
  vector<RoadPoint> ftPoints;
  if (route.size() < 2)
  {
    if (route.size() == 1)
      ftPoints.emplace_back(m_jointIndex.GetFtPoint(route[0]));
    return ftPoints;
  }

  ftPoints.reserve(route.size() * 2);

  for (size_t i = 0; i < route.size() - 1; ++i)
  {
    Joint::Id const prevJoint = route[i];
    Joint::Id const nextJoint = route[i + 1];

    auto const & pair = m_jointIndex.FindCommonFeature(prevJoint, nextJoint);
    if (i == 0)
      ftPoints.push_back(pair.first);

    uint32_t const featureId = pair.first.GetFeatureId();
    uint32_t const segFrom = pair.first.GetPointId();
    uint32_t const segTo = pair.second.GetPointId();

    if (segFrom < segTo)
    {
      for (uint32_t seg = segFrom + 1; seg < segTo; ++seg)
        ftPoints.push_back({featureId, seg});
    }
    else if (segFrom > segTo)
    {
      for (uint32_t seg = segFrom - 1; seg > segTo; --seg)
        ftPoints.push_back({featureId, seg});
    }
    else
      MYTHROW(RootException,
              ("Wrong equality segFrom = segTo =", segFrom, ", featureId = ", featureId));

    ftPoints.push_back(pair.second);
  }

  return ftPoints;
}

inline void IndexGraph::AddNeighboringEdge(RoadGeometry const & road, RoadPoint rp, bool forward,
                                           vector<TEdgeType> & edges) const
{
  pair<Joint::Id, uint32_t> const & pair = m_roadIndex.FindNeighbor(rp, forward);
  if (pair.first != Joint::kInvalidId)
  {
    double const distance = m_estimator->CalcEdgesWeight(road, rp.GetPointId(), pair.second);
    edges.push_back({pair.first, distance});
  }
}

inline void IndexGraph::GetEdgesList(Joint::Id jointId, bool forward,
                                     vector<TEdgeType> & edges) const
{
  edges.clear();

  m_jointIndex.ForEachPoint(jointId, [this, &edges, forward](RoadPoint const & rp) {
    RoadGeometry const & road = m_geometry.GetRoad(rp.GetFeatureId());
    if (!road.IsRoad())
      return;

    bool const twoWay = !road.IsOneWay();
    if (!forward || twoWay)
      AddNeighboringEdge(road, rp, false, edges);

    if (forward || twoWay)
      AddNeighboringEdge(road, rp, true, edges);
  });
}
}  // namespace routing
