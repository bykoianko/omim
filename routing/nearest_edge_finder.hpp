#pragma once

#include "routing/road_graph.hpp"

#include "routing_common/vehicle_model.hpp"

#include "geometry/point2d.hpp"

#include "indexer/feature_altitude.hpp"
#include "indexer/feature_decl.hpp"
#include "indexer/mwm_set.hpp"

#include <cstdint>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace routing
{

/// Helper functor class to filter nearest roads to the given starting point.
/// Class returns pairs of outgoing edge and projection point on the edge
class NearestEdgeFinder
{
  struct Candidate
  {
    static auto constexpr kInvalidSegmentId = std::numeric_limits<uint32_t>::max();

    double m_squaredDist = std::numeric_limits<double>::max();
    uint32_t m_segId = kInvalidSegmentId;
    Junction m_segStart;
    Junction m_segEnd;
    Junction m_projPoint;
    FeatureID m_fid;
    bool m_bidirectional = true;
  };

  m2::PointD const m_point;
  std::vector<Candidate> m_candidates;

public:
  explicit NearestEdgeFinder(m2::PointD const & point);

  inline bool HasCandidates() const { return !m_candidates.empty(); }

  void AddInformationSource(FeatureID const & featureId, IRoadGraph::JunctionVec const & junctions,
                            bool bidirectional);

  void MakeResult(std::vector<std::pair<Edge, Junction>> & res, size_t maxCountFeatures);

private:
  void CandidateToResult(Candidate const & candidate, size_t maxCountFeatures,
                         std::vector<std::pair<Edge, Junction>> & res) const;
};
}  // namespace routing
