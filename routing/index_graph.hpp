#pragma once

#include "routing/edge_estimator.hpp"
#include "routing/geometry.hpp"
#include "routing/joint.hpp"
#include "routing/joint_index.hpp"
#include "routing/road_index.hpp"
#include "routing/road_point.hpp"
#include "routing/routing_serialization.hpp"

#include "coding/reader.hpp"
#include "coding/write_to_sink.hpp"

#include "geometry/point2d.hpp"

#include "std/cstdint.hpp"
#include "std/set.hpp"
#include "std/shared_ptr.hpp"
#include "std/unique_ptr.hpp"
#include "std/utility.hpp"
#include "std/vector.hpp"

namespace routing
{
class JointEdge final
{
public:
  JointEdge(Joint::Id target, double weight) : m_target(target), m_weight(weight) {}
  Joint::Id GetTarget() const { return m_target; }
  double GetWeight() const { return m_weight; }

private:
  // Target is vertex going to for outgoing edges, vertex going from for ingoing edges.
  Joint::Id m_target;
  double m_weight;
};

class IndexGraph final
{
public:
  // AStarAlgorithm types aliases:
  using TVertexType = Joint::Id;
  using TEdgeType = JointEdge;

  IndexGraph() = default;
  explicit IndexGraph(unique_ptr<GeometryLoader> loader, shared_ptr<EdgeEstimator> estimator);

  // AStarAlgorithm<TGraph> overloads:
  void GetOutgoingEdgesList(Joint::Id vertex, vector<JointEdge> & edges) const;
  void GetIngoingEdgesList(Joint::Id vertex, vector<JointEdge> & edges) const;
  double HeuristicCostEstimate(Joint::Id from, Joint::Id to) const;

  Geometry const & GetGeometry() const { return m_geometry; }
  m2::PointD const & GetPoint(RoadPoint const & ftp) const;
  m2::PointD const & GetPoint(Joint::Id jointId) const;
  size_t GetNumRoads() const { return m_roadIndex.GetSize(); }
  size_t GetNumJoints() const { return m_jointIndex.GetNumJoints(); }
  size_t GetNumPoints() const { return m_jointIndex.GetNumPoints(); }
  void Import(vector<Joint> const & joints);
  Joint::Id InsertJoint(RoadPoint const & rp);

  // Add intermediate points to route (those don't correspond to any joint).
  //
  // Also convert joint ids to RoadPoints.
  void RedressRoute(vector<Joint::Id> const & route, vector<RoadPoint> & roadPoints) const;

  template <class Sink>
  void Serialize(Sink & sink) const
  {
    WriteToSink(sink, static_cast<uint32_t>(GetNumJoints()));
    m_roadIndex.Serialize(sink);
  }

  template <class Source>
  void Deserialize(Source & src)
  {
    uint32_t const jointsSize = ReadPrimitiveFromSource<uint32_t>(src);
    m_roadIndex.Deserialize(src);
    Build(jointsSize);
  }

  Joint::Id GetJointIdForTesting(RoadPoint const & ftp) const {return m_roadIndex.GetJointId(ftp); }

  /// \brief Disable edge between |from| and |to| if they are different and adjacent.
  /// \note The method doesn't affect routing if |from| and |to| are not adjacent or
  /// if one of them is equal to Joint::kInvalidId.
  void DisableEdge(Joint::Id from, Joint::Id to) { m_blockedEdges.insert(make_pair(from, to)); }

  /// \brief Connects joint |from| and |to| with a fake oneway feature. Geometry for end of
  /// the feature is taken form first point of |from| and |to|. If |viaPointGeometry| is not empty
  /// the feature is created with intermediate (not joint) point(s).
  /// \returns feature id which was added.
  uint32_t AddFakeFeature(Joint::Id from, Joint::Id to, vector<RoadPoint> const & viaPointGeometry);

  /// \brief Adds restriction to navigation graph which says that it's prohibited to go from
  /// |from| to |to|.
  /// \note |from| and |to| could be only begining or ending feature points and they has to belong to
  /// the same junction with |jointId|. That means features |from| and |to| has to be adjacent.
  /// \note This method could be called only after |m_roads| have been loaded with the help of Deserialize()
  /// or Import().
  /// \note This method adds fake features with ids which follows immediately after real feature ids.
  void ApplyRestrictionNo(routing::RoadPoint const & from, routing::RoadPoint const & to, Joint::Id jointId);

  /// \brief Adds restriction to navigation graph which says that from feature |from| it's permitted only
  /// to go to feature |to| all other ways starting form |form| is prohibited.
  /// \note All notes which are valid for ApplyRestrictionNo() is valid for ApplyRestrictionOnly().
  void ApplyRestrictionOnly(routing::RoadPoint const & from, routing::RoadPoint const & to, Joint::Id jointId);

  /// \brief Add restrictions in |restrictions| to |m_ftPointIndex|.
  void ApplyRestrictions(RestrictionVec const & restrictions);

  /// \brief Creates a list of RoadPoint from joint |from| to joint |to| if |from| and
  /// |to| belongs to one feature. Points |from| and |to| are included in |roadPoints|.
  /// The order on points in |roadPoints| is from |from| to |to|.
  void CreateRoadPointsList(Joint::Id from, Joint::Id to, vector<RoadPoint> & roadPoints);

  /// \brief Fills |roadPoints| with points of path composed of |newFeatureJoints|.
  /// Removes the begining point and the end point of the path.
  void GetIntermediatePoints(vector<Joint::Id> const & newFeatureJoints,
                             vector<RoadPoint> & roadPoints,
                             vector<size_t> & internalJointIdx);

  void CreateFakeFeatureGeometry(vector<RoadPoint> const & geometrySource, RoadGeometry & geometry) const;

  /// \returns RoadGeometry by a real or fake featureId.
  RoadGeometry const & GetRoad(uint32_t featureId) const;

  static uint32_t const kStartFakeFeatureIds = 1024 * 1024 * 1024;

private:
  void AddNeighboringEdge(RoadGeometry const & road, RoadPoint rp, bool forward,
                          bool outgoing, vector<TEdgeType> & edges) const;
  void GetEdgesList(Joint::Id jointId, bool forward, vector<TEdgeType> & edges) const;
  void Build(uint32_t jointNumber);

  double GetSpeed(RoadPoint ftp) const;

  /// \brief Finds neghboring of |centerId| joint on feature id of |center|
  /// which is contained in |edges| and fills |oneStepAside| with it.
  /// \note If oneStepAside is empty no neghboring nodes were found.
  /// \note Taking into account the way of setting restrictions almost always |oneStepAside|
  /// will contain one or zero items. Besides the it it's posible to draw map
  /// the (wrong) way |oneStepAside| will contain any number of items.
  void FindOneStepAsideRoadPoint(RoadPoint const & center, Joint::Id centerId,
                                 vector<TEdgeType> const & edges, vector<Joint::Id> & oneStepAside) const;

  bool ApplyRestrictionPrepareData(RoadPoint const & from, RoadPoint const & to, Joint::Id centerId,
                                   vector<TEdgeType> & ingoingEdges, vector<TEdgeType> & outgoingEdges,
                                   Joint::Id & fromFirstOneStepAside, Joint::Id & toFirstOneStepAside);

  Geometry m_geometry;
  shared_ptr<EdgeEstimator> m_estimator;
  RoadIndex m_roadIndex;
  JointIndex m_jointIndex;

  set<pair<Joint::Id, Joint::Id>> m_blockedEdges;
  uint32_t m_nextFakeFeatureId = kStartFakeFeatureIds;
  // Mapping from fake feature id to fake feature geometry.
  map<uint32_t, RoadGeometry> m_fakeFeatureGeometry;
};
}  // namespace routing
