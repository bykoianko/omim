#include "routing/cross_mwm_index_graph_osrm.hpp"

namespace routing
{
bool CrossMwmIndexGraphOsrm::IsTransition(Segment const & s, bool isOutgoing) const
{
  return true;
}

void CrossMwmIndexGraphOsrm::GetTwin(Segment const & s, std::vector<Segment> & twins) const
{

}

void CrossMwmIndexGraphOsrm::GetEdgeList(Segment const & s,
                                         bool isOutgoing, std::vector<SegmentEdge> & edges) const
{

}
}  // namespace routing
