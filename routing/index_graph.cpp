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

uint32_t IndexGraph::AddFakeFeature(Joint::Id from, Joint::Id to, vector<RoadPoint> const & viaPointGeometry)
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

  // Adding to indexes for fake feature: |from|->{via point}->|to|.
  RoadPoint const fromFakeFtPoint(m_nextFakeFeatureId, 0 /* Before via points */);
  RoadPoint const toFakeFtPoint(m_nextFakeFeatureId, viaPointGeometry.size() + 1/* After via points */);

  m_roadIndex.AddJoint(fromFakeFtPoint, from);
  m_roadIndex.AddJoint(toFakeFtPoint, to);

  m_jointIndex.AppendJoint(from, fromFakeFtPoint);
  m_jointIndex.AppendJoint(to, toFakeFtPoint);

  return m_nextFakeFeatureId++;
}

void IndexGraph::FindFirstOneStepAsideRoadPoint(RoadPoint const & center, Joint::Id centerId,
                                                vector<TEdgeType> const & edges,
                                                vector<Joint::Id> & oneStepAside) const
{
  oneStepAside.clear();
  m_roadIndex.ForEachJoint(center.GetFeatureId(),
                           [&](uint32_t /*pointId*/, Joint::Id jointId){
    for (TEdgeType const & e : edges)
    {
      if (e.GetTarget() == jointId)
        oneStepAside.push_back(jointId);
    }
  });
}

bool IndexGraph::ApplyRestrictionPrepareData(RoadPoint const & from, RoadPoint const & to,
                                             Joint::Id centerId,
                                             vector<TEdgeType> & ingoingEdges, vector<TEdgeType> & outgoingEdges,
                                             Joint::Id & fromFirstOneStepAside, Joint::Id & toFirstOneStepAside)
{
  GetIngoingEdgesList(centerId, ingoingEdges);
  vector<Joint::Id> fromOneStepAside;
  FindFirstOneStepAsideRoadPoint(from, centerId, ingoingEdges, fromOneStepAside);
  if (fromOneStepAside.empty())
    return false;

  GetOutgoingEdgesList(centerId, outgoingEdges);
  vector<Joint::Id> toOneStepAside;
  FindFirstOneStepAsideRoadPoint(to, centerId, outgoingEdges, toOneStepAside);
  if (toOneStepAside.empty())
    return false;

  fromFirstOneStepAside = fromOneStepAside.back();
  toFirstOneStepAside = toOneStepAside.back();
  return true;
}

void IndexGraph::ApplyRestrictionNo(RoadPoint const & from, RoadPoint const & to,
                                    Joint::Id centerId)
{
  vector<TEdgeType> ingoingEdges;
  vector<TEdgeType> outgoingEdges;
  Joint::Id fromFirstOneStepAside = Joint::kInvalidId;
  Joint::Id toFirstOneStepAside = Joint::kInvalidId;
  if (!ApplyRestrictionPrepareData(from, to, centerId, ingoingEdges, outgoingEdges,
                                   fromFirstOneStepAside, toFirstOneStepAside))
  {
    return;
  }

  // One ingoing edge case.
  if (ingoingEdges.size() == 1)
  {
    DisableEdge(centerId, toFirstOneStepAside);
    return;
  }

  // One outgoing edge case.
  if (outgoingEdges.size() == 1)
  {
    DisableEdge(fromFirstOneStepAside, centerId);
    return;
  }

  // Prohibition of moving from one segment to another in case of any number of ingoing and outgoing edges.
  // The idea is to tranform the navigation graph for every non-degenerate case as it's shown below.
  // At the picture below a restriction for prohibition moving from 4 to O to 3 is shown.
  // So to implement it it's necessary to remove (disable) an edge 4-O and add features (edges) 4-N-1 and N-2.
  //
  // 1       2       3                     1       2       3
  // *       *       *                     *       *       *
  //  ↖     ^     ↗                       ^↖   ↗^     ↗
  //    ↖   |   ↗                         |  ↖   |   ↗
  //      ↖ | ↗                           |↗   ↖| ↗
  //         *  O             ==>        N *       *  O
  //      ↗ ^ ↖                           ^       ^ ↖
  //    ↗   |   ↖                         |       |   ↖
  //  ↗     |     ↖                       |       |     ↖
  // *       *       *                     *       *       *
  // 4       5       7                     4       5       7

  DisableEdge(fromFirstOneStepAside, centerId);
  outgoingEdges.erase(remove_if(outgoingEdges.begin(), outgoingEdges.end(),
                                [&toFirstOneStepAside](TEdgeType const & e){
                        return e.GetTarget() == toFirstOneStepAside;
                      }), outgoingEdges.end());
  Joint::Id newJoint = Joint::kInvalidId;
  for (auto it = outgoingEdges.cbegin(); it != outgoingEdges.cend(); ++it)
  {
    if (it == outgoingEdges.cbegin())
    {
      uint32_t const featureId = AddFakeFeature(fromFirstOneStepAside, it->GetTarget(), {from});
      newJoint = InsertJoint({featureId, 1 /* Intermediate point of added feature */});
    }
    else
    {
      AddFakeFeature(newJoint, it->GetTarget(), {} /* via points */);
    }
  }
}

void IndexGraph::ApplyRestrictionOnly(RoadPoint const & from, RoadPoint const & to,
                                      Joint::Id centerId)
{
  vector<TEdgeType> ingoingEdges;
  vector<TEdgeType> outgoingEdges;
  Joint::Id fromFirstOneStepAside = Joint::kInvalidId;
  Joint::Id toFirstOneStepAside = Joint::kInvalidId;
  if (!ApplyRestrictionPrepareData(from, to, centerId, ingoingEdges, outgoingEdges,
                                   fromFirstOneStepAside, toFirstOneStepAside))
  {
    return;
  }

  // One ingoing edge case.
  if (ingoingEdges.size() == 1)
  {
    DisableEdge(fromFirstOneStepAside, centerId);
    return;
  }

  // One outgoing edge case.
  if (outgoingEdges.size() == 1)
  {
    DisableEdge(centerId, toFirstOneStepAside);
    return;
  }

  // It's possible to move only from one segment to another in case of any number of ingoing and outgoing edges.
  // The idea is to tranform the navigation graph for every non-degenerate case as it's shown below.
  // At the picture below a restriction for permission moving only from 7 to O to 3 is shown.
  // So to implement it it's necessary to remove (disable) an edge 7-O and add feature (edge) 4-N-3.
  // Adding N is important for a route recovery stage. (The geometry of O will be copied to N.)
  //
  // 1       2       3                     1       2       3
  // *       *       *                     *       *       *
  //  ↖     ^     ↗                        ↖     ^     ↗^
  //    ↖   |   ↗                            ↖   |   ↗  |
  //      ↖ | ↗                                ↖| ↗     |
  //         *  O             ==>                  *  O    * N
  //      ↗ ^ ↖                                 ↗^       ^
  //    ↗   |   ↖                             ↗  |       |
  //  ↗     |     ↖                         ↗    |       |
  // *       *       *                     *       *       *
  // 4       5       7                     4       5       7

  DisableEdge(fromFirstOneStepAside, centerId);
  AddFakeFeature(fromFirstOneStepAside, toFirstOneStepAside, {from});
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
                                           bool outgoing, vector<TEdgeType> & edges) const
{
  pair<Joint::Id, uint32_t> const & neighbor = m_roadIndex.FindNeighbor(rp, forward);
  if (neighbor.first == Joint::kInvalidId)
    return;

  Joint::Id const rbJointId = m_roadIndex.GetJointId(rp);
  auto const edge = outgoing ? make_pair(rbJointId, neighbor.first)
                             : make_pair(neighbor.first, rbJointId);
  if (m_blockedEdges.find(edge) != m_blockedEdges.end())
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
      AddNeighboringEdge(road, rp, false, forward, edges);

    if (forward || twoWay)
      AddNeighboringEdge(road, rp, true, forward, edges);
  });
}
}  // namespace routing
