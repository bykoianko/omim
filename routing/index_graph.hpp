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
  Joint::Id const m_target;
  double const m_weight;
};

class IndexGraph final
{
public:
  using TVertexType = Joint::Id;
  using TEdgeType = JointEdge;

  IndexGraph() = default;
  explicit IndexGraph(unique_ptr<GeometryLoader> loader, shared_ptr<EdgeEstimator> estimator);

  // AStarAlgorithm<TGraph> overloads:
  void GetOutgoingEdgesList(TVertexType vertex, vector<TEdgeType> & edges) const;
  void GetIngoingEdgesList(TVertexType vertex, vector<TEdgeType> & edges) const;
  double HeuristicCostEstimate(TVertexType from, TVertexType to) const;

  Geometry const & GetGeometry() const { return m_geometry; }
  m2::PointD const & GetPoint(RoadPoint const & ftp) const;
  m2::PointD const & GetPoint(Joint::Id jointId) const;
  size_t GetNumRoads() const { return m_roadIndex.GetSize(); }
  size_t GetNumJoints() const { return m_jointIndex.GetNumJoints(); }
  size_t GetNumPoints() const { return m_jointIndex.GetNumPoints(); }
  void Import(vector<Joint> const & joints);
  Joint::Id InsertJoint(RoadPoint const & rp);
  vector<RoadPoint> RedressRoute(vector<Joint::Id> const & route) const;
  void Import(vector<Joint> const & joints, RestrictionVec const & restrictions);

  template <class Sink>
  void Serialize(Sink & sink) const
  {
    WriteToSink(sink, static_cast<Joint::Id>(GetNumJoints()));
    m_roadIndex.Serialize(sink);
  }

  template <class Source>
  void Deserialize(Source & src, RestrictionVec const & restrictions)
  {
    uint32_t const jointsSize = ReadPrimitiveFromSource<uint32_t>(src);
    m_roadIndex.Deserialize(src);
    Build(jointsSize, restrictions);
  }

  Joint::Id GetJointIdForTesting(RoadPoint const & ftp) const {return m_roadIndex.GetJointId(ftp); }

  /// \brief Disable edge between |from| and |to| if they are different and adjacent.
  /// \note The method doesn't affect routing if |from| and |to| are not adjacent or
  /// if one of them is equal to Joint::kInvalidId.
  void DisableEdge(Joint::Id from, Joint::Id to) { m_blockedEdges.insert(make_pair(from, to)); }

  /// \brief Connects joint |from| and |to| with a fake oneway feature. Geometry for end of
  /// the feature is taken form first point of |from| and |to|. If |viaPointGeometry| is not empty
  /// the feature is created with intermediate (not joint) point(s).
  void AddFakeFeature(Joint::Id from, Joint::Id to, vector<RoadPoint> const & viaPointGeometry);

  /// \brief Connects point |form| and joint |to| with a one segment, oneway fake feature.
  /// If |form| corresponds a joint, the joint and |to| are connected with a fake feature.
  /// If not, creates a joint for |from| and then connects it with |to|.
  void AddFakeFeature(RoadPoint const & from, Joint::Id to);

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

  static uint32_t const kStartFakeFeatureIds = 1024 * 1024 * 1024;

private:
  void AddNeighboringEdge(RoadGeometry const & road, RoadPoint rp, bool forward,
                          vector<TEdgeType> & edges) const;
  void GetEdgesList(Joint::Id jointId, bool forward, vector<TEdgeType> & edges) const;
  void Build(uint32_t jointNumber, RestrictionVec const & restrictions);

  /// \returns RoadGeometry by a real or fake featureId.
  RoadGeometry const & GetRoad(uint32_t featureId) const;

  void CreateFakeFeatureGeometry(vector<RoadPoint> const & geometrySource, RoadGeometry & geometry) const;

  double GetSpeed(RoadPoint ftp) const;

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
