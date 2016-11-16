#pragma once

#include "routing/edge_estimator.hpp"
#include "routing/fseg.hpp"
#include "routing/fseg_index.hpp"
#include "routing/geometry.hpp"
#include "routing/joint.hpp"
#include "routing/joint_index.hpp"

#include "coding/reader.hpp"
#include "coding/write_to_sink.hpp"

#include "geometry/point2d.hpp"

#include "std/cstdint.hpp"
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
  m2::PointD const & GetPoint(Joint::Id jointId) const;
  size_t GetNumRoads() const { return m_fsegIndex.GetSize(); }
  size_t GetNumJoints() const { return m_jointIndex.GetNumJoints(); }
  size_t GetNumFtPoints() const { return m_jointIndex.GetNumFtPoints(); }
  void Import(vector<Joint> const & joints);
  Joint::Id InsertJoint(FSegId const & fseg);
  vector<FSegId> RedressRoute(vector<Joint::Id> const & route) const;

  template <class TSink>
  void Serialize(TSink & sink) const
  {
    WriteToSink(sink, static_cast<Joint::Id>(GetNumJoints()));
    m_fsegIndex.Serialize(sink);
  }

  template <class TSource>
  void Deserialize(TSource & src)
  {
    uint32_t const jointsSize = ReadPrimitiveFromSource<uint32_t>(src);
    m_fsegIndex.Deserialize(src);
    m_jointIndex.Build(m_fsegIndex, jointsSize);
  }

private:
  void AddNeighboringEdge(RoadGeometry const & road, FSegId fseg, bool forward,
                          vector<TEdgeType> & edges) const;
  void GetEdgesList(Joint::Id jointId, bool forward, vector<TEdgeType> & edges) const;

  Geometry m_geometry;
  shared_ptr<EdgeEstimator> m_estimator;
  FSegIndex m_fsegIndex;
  JointIndex m_jointIndex;
};
}  // namespace routing
