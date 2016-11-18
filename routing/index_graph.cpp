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

m2::PointD const & IndexGraph::GetPoint(RoadPoint const & ftp) const
{
  RoadGeometry const & road = GetRoad(ftp.GetFeatureId());
  CHECK_LESS(ftp.GetPointId(), road.GetPointsCount(), ());
  return road.GetPoint(ftp.GetPointId());
}

m2::PointD const & IndexGraph::GetPoint(Joint::Id jointId) const
{
  return GetPoint(m_jointIndex.GetPoint(jointId));
}

double IndexGraph::GetSpeed(RoadPoint ftp) const
{
  return GetRoad(ftp.GetFeatureId()).GetSpeed();
}

void IndexGraph::Build(uint32_t jointNumber, RestrictionVec const & restrictions)
{
  m_jointIndex.Build(m_roadIndex, jointNumber);
  ApplyRestrictions(restrictions);
}

void IndexGraph::Import(vector<Joint> const & joints, RestrictionVec const & restrictions)
{
  m_roadIndex.Import(joints);
  Build(joints.size(), restrictions);
}

void IndexGraph::CreateFakeFeatureGeometry(vector<RoadPoint> const & geometrySource,
                                           RoadGeometry & geometry) const
{
  double averageSpeed = 0.0;
  buffer_vector<m2::PointD, 32> points(geometrySource.size());
  for (size_t i = 0; i < geometrySource.size(); ++i)
  {
    RoadPoint const ftp = geometrySource[i];
    averageSpeed += GetSpeed(ftp) / geometrySource.size();
    points[i] = GetPoint(ftp);
  }

  geometry = RoadGeometry(true /* oneWay */, averageSpeed, points);
}

void IndexGraph::AddFakeFeature(Joint::Id from, Joint::Id to, vector<RoadPoint> const & viaPointGeometry)
{
  CHECK_LESS(from, m_jointIndex.GetNumJoints(), ());
  CHECK_LESS(to, m_jointIndex.GetNumJoints(), ());

  // Getting fake feature geometry.
  RoadGeometry geom;
  vector<RoadPoint> geometrySource({m_jointIndex.GetPoint(from)});
  geometrySource.insert(geometrySource.end(), viaPointGeometry.cbegin(), viaPointGeometry.cend());
  geometrySource.push_back(m_jointIndex.GetPoint(to));
  CreateFakeFeatureGeometry(geometrySource, geom);
  m_fakeFeatureGeometry.insert(make_pair(m_nextFakeFeatureId, geom));

  // Adding to indexes.
  RoadPoint const fromFakeFtPoint(m_nextFakeFeatureId, 0);
  RoadPoint const toFakeFtPoint(m_nextFakeFeatureId, 1);

  m_roadIndex.AddJoint(fromFakeFtPoint, from);
  m_roadIndex.AddJoint(toFakeFtPoint, to);

  m_jointIndex.AppendJoint(from, fromFakeFtPoint);
  m_jointIndex.AppendJoint(to, toFakeFtPoint);

  ++m_nextFakeFeatureId;
}

void IndexGraph::AddFakeFeature(RoadPoint const & fromPnt, Joint::Id to)
{
  AddFakeFeature(InsertJoint(fromPnt), to, {} /* viaPointGeometry */);
}

void IndexGraph::ApplyRestrictionNo(RoadPoint const & from, RoadPoint const & to,
                                    Joint::Id centerId)
{
  auto findOneStepAsideRoadPoint = [&](RoadPoint const & center, vector<TEdgeType> const & edges,
      Joint::Id & oneStepAside)
  {
    auto const itOneStepAside = find_if(edges.cbegin(), edges.cend(), [&](TEdgeType const & e){
      // @TODO This place is possible to implement faster. It should be taken into account
      // if a profiler shows this place.
      return m_jointIndex.FindCommonFeature(centerId, e.GetTarget()).first.GetFeatureId() == center.GetFeatureId();
    });
    if (itOneStepAside == edges.cend())
      return false;
    oneStepAside = itOneStepAside->GetTarget();
    return true;
  };

  vector<TEdgeType> ingoingEdges;
  GetIngoingEdgesList(centerId, ingoingEdges);
  Joint::Id fromOneStepAside = Joint::kInvalidId;
  if (!findOneStepAsideRoadPoint(from, ingoingEdges, fromOneStepAside))
    return;

  vector<TEdgeType> outgoingEdges;
  GetOutgoingEdgesList(centerId, outgoingEdges);
  Joint::Id toOneStepAside = Joint::kInvalidId;
  if (!findOneStepAsideRoadPoint(to, outgoingEdges, toOneStepAside))
    return;

  // One ingoing edge case.
  if (ingoingEdges.size() == 1)
  {
    DisableEdge(centerId, toOneStepAside);
    return;
  }

  // One outgoing edge case.
  if (outgoingEdges.size() == 1)
  {
    DisableEdge(fromOneStepAside, centerId);
    return;
  }

  // Prohibition of moving from one segment to another in any number of ingoing and outgoing edges.
  // The idea is to tranform navigation graph...

}

void IndexGraph::ApplyRestrictionOnly(RoadPoint const & from, RoadPoint const & to,
                                      Joint::Id jointId)
{
}

void IndexGraph::ApplyRestrictions(RestrictionVec const & restrictions)
{
  for (Restriction const & restriction : restrictions)
  {
    if (restriction.m_featureIds.size() != 2)
    {
      LOG(LERROR, ("Only to link restriction are supported."));
      continue;
    }

    routing::RoadPoint from;
    routing::RoadPoint to;
    Joint::Id jointId;
    if (!m_roadIndex.GetAdjacentFtPoints(restriction.m_featureIds[0], restriction.m_featureIds[1],
        from, to, jointId))
    {
      continue; // Restriction is not contain adjacent features.
    }
    // @TODO(bykoianko) By knowing |from|, |to|, |jointId| and |restriction.m_type|
    // |m_geometry| should be edited.
    switch (restriction.m_type)
    {
    case Restriction::Type::No: ApplyRestrictionNo(from, to, jointId); continue;
    case Restriction::Type::Only: ApplyRestrictionOnly(from, to, jointId); continue;
    }
  }
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
      ftPoints.emplace_back(m_jointIndex.GetPoint(route[0]));
    return ftPoints;
  }

  ftPoints.reserve(route.size() * 2);

  for (size_t i = 0; i < route.size() - 1; ++i)
  {
    Joint::Id const prevJoint = route[i];
    Joint::Id const nextJoint = route[i + 1];

    RoadPoint rp0;
    RoadPoint rp1;
    m_jointIndex.FindCommonFeature(prevJoint, nextJoint, rp0, rp1);
    if (i == 0)
      ftPoints.push_back(rp0);

    uint32_t const featureId = rp0.GetFeatureId();
    uint32_t const segFrom = rp0.GetPointId();
    uint32_t const segTo = rp1.GetPointId();

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

    ftPoints.push_back(rp1);
  }

  return ftPoints;
}

inline void IndexGraph::AddNeighboringEdge(RoadGeometry const & road, RoadPoint rp, bool forward,
                                           vector<TEdgeType> & edges) const
{
  pair<Joint::Id, uint32_t> const & neighbor = m_roadIndex.FindNeighbor(rp, forward);
  if (neighbor.first == Joint::kInvalidId)
    return;

  Joint::Id const from = m_roadIndex.GetJointId(rp);
  if (m_blockedEdges.find(make_pair(from, neighbor.first)) != m_blockedEdges.end())
    return;

  double const distance = m_estimator->CalcEdgesWeight(road, rp.GetPointId(), neighbor.second);
  edges.push_back({neighbor.first, distance});
}

RoadGeometry const & IndexGraph::GetRoad(uint32_t featureId) const
{
  auto const it = m_fakeFeatureGeometry.find(featureId);
  if (it != m_fakeFeatureGeometry.cend())
    return it->second;

  return m_geometry.GetRoad(featureId);
}

inline void IndexGraph::GetEdgesList(Joint::Id jointId, bool forward,
                                     vector<TEdgeType> & edges) const
{
  edges.clear();

  m_jointIndex.ForEachPoint(jointId, [this, &edges, forward](RoadPoint const & rp) {
    RoadGeometry const & road = GetRoad(rp.GetFeatureId());
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
