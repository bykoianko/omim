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
}  // namespace routing
