#pragma once

#include "routing/cross_mwm_index_graph.hpp"
#include "routing/routing_mapping.hpp"

namespace routing
{
class CrossMwmIndexGraphOsrm : public CrossMwmIndexGraph
{
public:
  CrossMwmIndexGraphOsrm(RoutingIndexManager & indexManager) : m_indexManager(indexManager) {}

  //  CrossMwmIndexGraph overrides:
  bool IsTransition(Segment const & s, bool isOutgoing) const override;
  void GetTwin(Segment const & s, std::vector<Segment> & twins) const override;
  void GetEdgeList(Segment const & s, bool isOutgoing, std::vector<SegmentEdge> & edges) const override;

private:
  RoutingIndexManager & m_indexManager;
};
}  // namespace routing
