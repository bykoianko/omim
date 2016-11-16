#pragma once

#include "routing/ftpoint.hpp"

#include "base/buffer_vector.hpp"

#include "std/cstdint.hpp"
#include "std/limits.hpp"

namespace routing
{
// Joint represents roads connection.
// It contains feature id, segment id for each road connected.
class Joint final
{
public:
  using Id = uint32_t;
  static Id constexpr kInvalidId = numeric_limits<Id>::max();

  void AddEntry(FtPoint entry) { m_entries.emplace_back(entry); }

  size_t GetSize() const { return m_entries.size(); }

  FtPoint const & GetEntry(size_t i) const { return m_entries[i]; }

private:
  buffer_vector<FtPoint, 2> m_entries;
};
}  // namespace routing
