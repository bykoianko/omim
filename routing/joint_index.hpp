#pragma once

#include "routing/fseg.hpp"
#include "routing/fseg_index.hpp"
#include "routing/joint.hpp"

#include "std/vector.hpp"

namespace routing
{
// JointIndex contains mapping from JointId to FtSegs.
//
// It is vector<Joint> conceptually.
// Technically Joint entries are joined into the single vector to reduce allocations overheads.
class JointIndex final
{
public:
  size_t GetJointsAmount() const { return m_slices.size(); }
  size_t GetFSegsAmount() const { return m_fsegs.size(); }
  FSegId GetFSeg(JointId jointId) const { return m_fsegs[GetSlice(jointId).Begin()]; }

  template <typename F>
  void ForEachFtSeg(JointId jointId, F && f) const
  {
    Slice const & slice = GetSlice(jointId);
    for (size_t i = slice.Begin(); i < slice.End(); ++i)
      f(m_fsegs[i]);
  }

  void Build(FSegIndex const & fsegIndex, uint32_t jointsAmount);
  pair<FSegId, FSegId> FindCommonFeature(JointId jointId0, JointId jointId1) const;
  JointId InsertJoint(FSegId const & fseg);

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

  Slice const & GetSlice(JointId jointId) const
  {
    ASSERT_LESS(jointId, m_slices.size(), ("JointId out of bounds"));
    return m_slices[jointId];
  }

  vector<Slice> m_slices;
  vector<FSegId> m_fsegs;
};
}  // namespace routing
