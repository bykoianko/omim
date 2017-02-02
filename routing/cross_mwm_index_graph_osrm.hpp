#pragma once

#include "routing/cross_mwm_index_graph.hpp"
#include "routing/routing_mapping.hpp"

#include "storage/num_mwm_id.hpp"

#include "base/math.hpp"

#include <unordered_set>
#include <utility>

namespace routing
{
class CrossMwmIndexGraphOsrm : public CrossMwmIndexGraph
{
public:
  CrossMwmIndexGraphOsrm(storage::NumMwmIds & numMwmIds, RoutingIndexManager & indexManager)
    : m_indexManager(indexManager), m_numMwmIds(numMwmIds) {}

  // CrossMwmIndexGraph overrides:
  bool IsTransition(Segment const & s, bool isOutgoing) const override;
  void GetTwin(Segment const & s, std::vector<Segment> & twins) const override;
  void GetEdgeList(Segment const & s, bool isOutgoing, std::vector<SegmentEdge> & edges) const override;

private:
  struct SegAndOutgoingFlag
  {
    Segment m_seg;
    bool m_isOutgoing;

    bool operator==(SegAndOutgoingFlag const & value) const
    {
      return m_seg == value.m_seg && m_isOutgoing == value.m_isOutgoing;
    }
  };

  struct HashSegAndOutgoingFlag
  {
    size_t operator()(SegAndOutgoingFlag const & p) const
    {
      return my::Hash(my::Hash(my::Hash(my::Hash(p.m_seg.IsForward(), p.m_seg.GetMwmId()),
                      p.m_seg.GetFeatureId()), p.m_seg.GetSegmentIdx()), p.m_isOutgoing);
    }
  };

  RoutingIndexManager & m_indexManager;
  storage::NumMwmIds & m_numMwmIds;

  std::unordered_map<SegAndOutgoingFlag, int, HashSegAndOutgoingFlag> m_transitionCache;
};
}  // namespace routing
