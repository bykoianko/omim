#pragma once

#include "routing/road_point.hpp"

#include "base/buffer_vector.hpp"

#include "std/cstdint.hpp"
#include "std/limits.hpp"

namespace routing
{
// Joint represents roads connection.
// It contains RoadPoint for each connected road.
class Joint final
{
public:
  using Id = uint32_t;
  static Id constexpr kInvalidId = numeric_limits<Id>::max();

  void AddEntry(RoadPoint entry) { m_entries.emplace_back(entry); }

  size_t GetSize() const { return m_entries.size(); }

  RoadPoint const & GetEntry(size_t i) const { return m_entries[i]; }

private:
  buffer_vector<RoadPoint, 2> m_entries;
};
}  // namespace routing
