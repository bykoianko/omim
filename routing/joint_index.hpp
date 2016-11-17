#pragma once

#include "routing/joint.hpp"
#include "routing/road_index.hpp"
#include "routing/road_point.hpp"

#include "std/vector.hpp"

namespace routing
{
// JointIndex contains mapping from Joint::Id to FtSegs.
//
// It is vector<Joint> conceptually.
// Technically Joint entries are joined into the single vector to reduce allocations overheads.
class JointIndex final
{
public:
  size_t GetNumJoints() const { return m_slices.size(); }
  size_t GetNumPoints() const { return m_points.size(); }
  RoadPoint GetFtPoint(Joint::Id jointId) const { return m_points[GetSlice(jointId).Begin()]; }

  template <typename F>
  void ForEachPoint(Joint::Id jointId, F && f) const
  {
    Slice const & slice = GetSlice(jointId);
    for (size_t i = slice.Begin(); i < slice.End(); ++i)
      f(m_points[i]);
  }

  void Build(RoadIndex const & ftPointIndex, uint32_t jointsAmount);
  pair<RoadPoint, RoadPoint> FindCommonFeature(Joint::Id jointId0, Joint::Id jointId1) const;
  Joint::Id InsertJoint(RoadPoint const & rp);

private:
  class Slice final
  {
  public:
    Slice() = default;

    Slice(uint32_t begin, uint32_t end) : m_begin(begin), m_end(end)
    {
      ASSERT_LESS_OR_EQUAL(begin, end, ());
    }

    uint32_t Begin() const { return m_begin; }

    uint32_t End() const { return m_end; }

    void Assign(uint32_t offset)
    {
      m_begin = offset;
      m_end = offset;
    }

    void IncSize() { ++m_end; }

  private:
    uint32_t m_begin = 0;
    uint32_t m_end = 0;
  };

  Slice const & GetSlice(Joint::Id jointId) const
  {
    ASSERT_LESS(jointId, m_slices.size(), ("Joint::Id out of bounds"));
    return m_slices[jointId];
  }

  vector<Slice> m_slices;
  vector<RoadPoint> m_points;
};
}  // namespace routing
