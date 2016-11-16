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

void IndexGraph::GetOutgoingEdgesList(Joint::Id jointId, vector<JointEdge> & edges) const
{
  GetEdgesList(jointId, true, edges);
}

void IndexGraph::GetIngoingEdgesList(Joint::Id jointId, vector<JointEdge> & edges) const
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

void IndexGraph::CreateRoadPointsList(Joint::Id from, Joint::Id to, vector<RoadPoint> & roadPoints)
{
  CHECK_NOT_EQUAL(from, Joint::kInvalidId, ());
  CHECK_NOT_EQUAL(to, Joint::kInvalidId, ());

  RoadPoint fromPoint;
  RoadPoint toPoint;

  m_jointIndex.FindPointsWithCommonFeature(from, to, fromPoint, toPoint);

  CHECK_EQUAL(fromPoint.GetFeatureId(), toPoint.GetFeatureId(), ());
  roadPoints.clear();
  if (fromPoint == toPoint)
  {
    roadPoints.push_back(fromPoint);
    return;
  }

  int shift = toPoint.GetPointId() > fromPoint.GetPointId() ? 1 : -1;
  for (int i = fromPoint.GetPointId(); i != toPoint.GetPointId(); i += shift)
    roadPoints.emplace_back(fromPoint.GetFeatureId(), i);
  roadPoints.emplace_back(fromPoint.GetFeatureId(), toPoint.GetPointId());
}

void IndexGraph::GetIntermediatePoints(vector<Joint::Id> const & newFeatureJoints,
                                       vector<RoadPoint> & roadPoints,
                                       vector<size_t> & internalJointIdx)
{
  CHECK_LESS(1, newFeatureJoints.size(), ());

  roadPoints.clear();
  internalJointIdx.clear();
  for (size_t i = 1; i < newFeatureJoints.size(); ++i)
  {
    vector<RoadPoint> oneEdgeRoadPoints;
    CreateRoadPointsList(newFeatureJoints[i - 1], newFeatureJoints[i], oneEdgeRoadPoints);

    if (oneEdgeRoadPoints.size() < 2)
      continue;
    oneEdgeRoadPoints.pop_back();
    roadPoints.insert(roadPoints.end(), oneEdgeRoadPoints.cbegin(), oneEdgeRoadPoints.cend());
    // After removing last item roadPoints.size() is an index of the joint will be added in next loop.
    // But we need to deduct one because of after the loop finishs the head of |roadPoints|
    // is removed.
    internalJointIdx.push_back(roadPoints.size() - 1);
  }
  if (!internalJointIdx.empty())
    internalJointIdx.pop_back();

  if (!roadPoints.empty())
    roadPoints.erase(roadPoints.cbegin());
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

  m_jointIndex.AppendToJoint(from, fromFakeFtPoint);
  m_jointIndex.AppendToJoint(to, toFakeFtPoint);

  return m_nextFakeFeatureId++;
}

void IndexGraph::FindOneStepAsideRoadPoint(RoadPoint const & center, Joint::Id centerId,
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

bool IndexGraph::ApplyRestrictionPrepareData(RoadPoint const & from, RoadPoint const & to, Joint::Id centerId,
                                             vector<TEdgeType> & ingoingEdges, vector<TEdgeType> & outgoingEdges,
                                             Joint::Id & fromFirstOneStepAside, Joint::Id & toFirstOneStepAside)
{
  GetIngoingEdgesList(centerId, ingoingEdges);
  vector<Joint::Id> fromOneStepAside;
  FindOneStepAsideRoadPoint(from, centerId, ingoingEdges, fromOneStepAside);
  if (fromOneStepAside.empty())
    return false;

  GetOutgoingEdgesList(centerId, outgoingEdges);
  vector<Joint::Id> toOneStepAside;
  FindOneStepAsideRoadPoint(to, centerId, outgoingEdges, toOneStepAside);
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

  outgoingEdges.erase(remove_if(outgoingEdges.begin(), outgoingEdges.end(),
                                [&](TEdgeType const & e)
  {
    // Removing edge N->3 in example above.
    return e.GetTarget() == toFirstOneStepAside
    // Preveting form adding in loop below
    // cycles |fromFirstOneStepAside|->|centerId|->|fromFirstOneStepAside|.
        || e.GetTarget() == fromFirstOneStepAside
    // Removing edges |centerId|->|centerId|.
        || e.GetTarget() == centerId;
  }), outgoingEdges.end());

  Joint::Id newJoint = Joint::kInvalidId;
  for (auto it = outgoingEdges.cbegin(); it != outgoingEdges.cend(); ++it)
  {
    vector<RoadPoint> geometrySource;
    vector<size_t> internalJointIdxs;
    if (it == outgoingEdges.cbegin())
    {
      // Adding the edge 4->N->1 in example above.
      if (fromFirstOneStepAside == centerId || centerId == it->GetTarget())
      {
        // @TODO(bykoianko) In rare cases it's posible that outgoing edges staring from |centerId|
        // contain as targets |centerId|. The same thing with ingoing edges.
        // The most likely it's a consequence of adding
        // restrictions with type no for some bidirectional roads. It's necessary to
        // investigate this case, to undestand the reasons of appearing such edges clearly,
        // prevent appearing of such edges and write unit tests on it.
        return;
      }
      GetIntermediatePoints(vector<Joint::Id>({fromFirstOneStepAside, centerId,
          it->GetTarget()}), geometrySource, internalJointIdxs);

      CHECK_EQUAL(internalJointIdxs.size(), 1, ());
      CHECK_LESS(internalJointIdxs.front(), geometrySource.size() + 2 /* beginnign and ending */, ());
      uint32_t const featureId = AddFakeFeature(fromFirstOneStepAside, it->GetTarget(), geometrySource);
      newJoint = InsertJoint({featureId,
                              static_cast<uint32_t>(internalJointIdxs[0]) /* Intermediate joint of added feature */});
    }
    else
    {
      // Adding the edge N->2 in example above.
      GetIntermediatePoints(vector<Joint::Id>({centerId, it->GetTarget()}),
                                 geometrySource, internalJointIdxs);
      AddFakeFeature(newJoint, it->GetTarget(), geometrySource);
    }
  }
  DisableEdge(fromFirstOneStepAside, centerId);
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

  // One outgoing edge case.
  if (outgoingEdges.size() == 1)
  {
    for (auto const & e : ingoingEdges)
    {
      if (e.GetTarget() != fromFirstOneStepAside)
        DisableEdge(e.GetTarget(), centerId);
    }
    return;
  }

  // One ingoing edge case.
  if (ingoingEdges.size() == 1)
  {
    for (auto const & e : outgoingEdges)
    {
      if (e.GetTarget() != toFirstOneStepAside)
        DisableEdge(centerId, e.GetTarget());
    }
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

  vector<RoadPoint> geometrySource;
  vector<size_t> internalJointIdxs;
  GetIntermediatePoints(vector<Joint::Id>({fromFirstOneStepAside, centerId,
      toFirstOneStepAside}), geometrySource, internalJointIdxs);

  AddFakeFeature(fromFirstOneStepAside, toFirstOneStepAside, geometrySource);
  DisableEdge(fromFirstOneStepAside, centerId);
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

    try
    {
      switch (restriction.m_type)
      {
      case Restriction::Type::No: ApplyRestrictionNo(from, to, jointId); continue;
      case Restriction::Type::Only: ApplyRestrictionOnly(from, to, jointId); continue;
      }
    }
    catch(RootException const & e)
    {
      LOG(LERROR, ("Exception while applying restrictions. Message:", e.Msg()));
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

void IndexGraph::RedressRoute(vector<Joint::Id> const & route, vector<RoadPoint> & roadPoints) const
{
  if (route.size() < 2)
  {
    if (route.size() == 1)
      roadPoints.emplace_back(m_jointIndex.GetPoint(route[0]));
    return;
  }

  roadPoints.reserve(route.size() * 2);

  for (size_t i = 0; i < route.size() - 1; ++i)
  {
    Joint::Id const prevJoint = route[i];
    Joint::Id const nextJoint = route[i + 1];

    RoadPoint rp0;
    RoadPoint rp1;
    m_jointIndex.FindPointsWithCommonFeature(prevJoint, nextJoint, rp0, rp1);
    if (i == 0)
      roadPoints.emplace_back(rp0);

    uint32_t const featureId = rp0.GetFeatureId();
    uint32_t const pointFrom = rp0.GetPointId();
    uint32_t const pointTo = rp1.GetPointId();

    if (pointFrom < pointTo)
    {
      for (uint32_t pointId = pointFrom + 1; pointId < pointTo; ++pointId)
        roadPoints.emplace_back(featureId, pointId);
    }
    else if (pointFrom > pointTo)
    {
      for (uint32_t pointId = pointFrom - 1; pointId > pointTo; --pointId)
        roadPoints.emplace_back(featureId, pointId);
    }
    else
    {
      MYTHROW(RootException,
              ("Wrong equality pointFrom = pointTo =", pointFrom, ", featureId = ", featureId));
    }

    roadPoints.emplace_back(rp1);
  }
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

inline void IndexGraph::GetEdgesList(Joint::Id jointId, bool isOutgoing,
                                     vector<TEdgeType> & edges) const
{
  edges.clear();

  m_jointIndex.ForEachPoint(jointId, [this, &edges, isOutgoing](RoadPoint const & rp) {
    RoadGeometry const & road = GetRoad(rp.GetFeatureId());
    if (!road.IsRoad())
      return;

    bool const bidirectional = !road.IsOneWay();
    if (!isOutgoing || bidirectional)
      AddNeighboringEdge(road, rp, false /* forward */, isOutgoing, edges);

    if (isOutgoing || bidirectional)
      AddNeighboringEdge(road, rp, true /* forward */, isOutgoing, edges);
  });
}
}  // namespace routing
