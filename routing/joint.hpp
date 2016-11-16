#pragma once

#include "routing/fseg.hpp"

#include "base/buffer_vector.hpp"

#include "std/cstdint.hpp"
#include "std/limits.hpp"

namespace routing
{
using JointId = uint32_t;
JointId constexpr kInvalidJointId = numeric_limits<JointId>::max();

// Joint represents roads connection.
// It contains feature id, segment id for each road connected.
class Joint final
{
public:
  void AddEntry(FSegId entry) { m_entries.emplace_back(entry); }

  size_t GetSize() const { return m_entries.size(); }

  FSegId const & GetEntry(size_t i) const { return m_entries[i]; }

private:
  buffer_vector<FSegId, 2> m_entries;
};

class JointOffset final
{
public:
  JointOffset() : m_begin(0), m_end(0) {}

  JointOffset(uint32_t begin, uint32_t end) : m_begin(begin), m_end(end)
  {
    ASSERT_LESS_OR_EQUAL(begin, end, ());
  }

  uint32_t Begin() const { return m_begin; }

  uint32_t End() const { return m_end; }

  uint32_t Size() const { return m_end - m_begin; }

  void Assign(uint32_t offset)
  {
    m_begin = offset;
    m_end = offset;
  }

  void IncSize() { ++m_end; }

private:
  uint32_t m_begin;
  uint32_t m_end;
};
}  // namespace routing
