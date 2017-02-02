#include "routing/cross_mwm_index_graph_osrm.hpp"

#include "platform/country_file.hpp"

#include <utility>

namespace routing
{
bool CrossMwmIndexGraphOsrm::IsTransition(Segment const & s, bool isOutgoing) const
{
  if (m_transitionCache.count({s, isOutgoing}) != 0)
    return true;

  platform::CountryFile const & countryFile = m_numMwmIds.GetFile(s.GetMwmId());
  TRoutingMappingPtr mappingPtr = m_indexManager.GetMappingByName(countryFile.GetName());
  RoutingMapping & mapping = *mappingPtr;
  TNodesList const nodesList = mapping.m_segMapping.GetNodeIdByFid(s.GetFeatureId());

  for (NodeID const n : nodesList)
  {
    auto const range = mapping.m_segMapping.GetSegmentsRange(n);
    OsrmMappingTypes::FtSeg seg;
    mapping.m_segMapping.GetSegmentByIndex(range.first, seg);
    if (!seg.IsValid())
      continue;
  }

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
