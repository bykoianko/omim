#include "routing/subway_graph.hpp"

#include "indexer/ftypes_matcher.hpp"
#include "indexer/scales.hpp"

#include "geometry/mercator.hpp"

#include "base/macros.hpp"

#include <limits>

namespace
{
double constexpr kLineSearchRadiusMeters = 100.0;
double constexpr kEquivalenceDistMeters = 3.0;
}  // namespace

namespace routing
{
void SubwayGraph::GetOutgoingEdgesList(TVertexType const & vertex, vector<TEdgeType> & edges)
{
  // @TODO Implement filling |edges| more carefully.
  // Getting point by |vertex|.
  MwmSet::MwmHandle handle = m_index.GetMwmHandleByCountryFile(m_numMwmIds->GetFile(vertex.GetMwmId()));
  Index::FeaturesLoaderGuard const guard(m_index, handle.GetId());
  FeatureType ft;
  if (!guard.GetFeatureByIndex(vertex.GetFeatureId(), ft))
  {
    LOG(LERROR, ("Feature can't be loaded:", vertex.GetFeatureId()));
    return;
  }
  ft.ParseGeometry(FeatureType::BEST_GEOMETRY);
  CHECK_LESS(vertex.GetPointId(), ft.GetPointsCount(), ());
  m2::PointD const & vertexPoint = ft.GetPoint(vertex.GetPointId());

  // Getting outgoing edges by |vertexPoint|.
  // Moving along the same feature.
  if (vertex.GetPointId() != 0)
  {
    edges.emplace_back(
        SubwayVertex(vertex.GetMwmId(), vertex.GetFeatureId(),
                     static_cast<uint32_t>(vertex.GetPointId() - 1), vertex.GetType()),
        1.0 /* weight */);
  }
  if (vertex.GetPointId() + 1 != ft.GetPointsCount())
  {
    edges.emplace_back(
        SubwayVertex(vertex.GetMwmId(), vertex.GetFeatureId(),
                     static_cast<uint32_t>(vertex.GetPointId() + 1), vertex.GetType()),
        1.0 /* weight */);
  }

  // Changing feature.
  auto const f = [&](FeatureType const & ft)
  {
    if (!IsValidRoad(ft))
      return;

    NumMwmId const ftNumMwmId = m_numMwmIds->GetId(ft.GetID().m_mwmId.GetInfo()->GetLocalFile().GetCountryFile());
    uint32_t const ftFeatureId = ft.GetID().m_index;
    if (ftNumMwmId == vertex.GetMwmId() && ftFeatureId == vertex.GetFeatureId())
      return;

    ft.ParseGeometry(FeatureType::BEST_GEOMETRY);
    ft.ParseMetadata();

    size_t const pointsCount = ft.GetPointsCount();
    SubwayType const ftType = ftypes::IsSubwayLineChecker::Instance()(ft) ? SubwayType::Line
                                                                          : SubwayType::Change;

    double closestDistMeters = numeric_limits<double>::max();
    SubwayEdge closestEdge;

    for (size_t i = 0; i < pointsCount; ++i)
    {
      double const distMeters = MercatorBounds::DistanceOnEarth(vertexPoint, ft.GetPoint(i));
      if (distMeters < kEquivalenceDistMeters && distMeters < closestDistMeters)
      {
        closestDistMeters = distMeters;
        // In case of changing feature an edge with zero length added.
        closestEdge =
            SubwayEdge(SubwayVertex(ftNumMwmId, ftFeatureId, static_cast<uint32_t>(i), ftType),
                       0.0 /* weight */);
      }
    }
    if (closestDistMeters != numeric_limits<double>::max())
      edges.push_back(closestEdge);
  };
  m_index.ForEachInRect(
    f, MercatorBounds::RectByCenterXYAndSizeInMeters(vertexPoint, kEquivalenceDistMeters),
    scales::GetUpperScale());
}

void SubwayGraph::GetIngoingEdgesList(TVertexType const & vertex, vector<TEdgeType> & edges)
{
  NOTIMPLEMENTED();
}

double SubwayGraph::HeuristicCostEstimate(TVertexType const & from, TVertexType const & to)
{
  return 0.0;
}

SubwayVertex SubwayGraph::GetNearestStation(m2::PointD const & point) const
{
  SubwayVertex closestVertex;
  double closestVertexDistMeters = std::numeric_limits<double>::max();

  auto const f = [&](FeatureType const & ft)
  {
    if (!IsValidRoad(ft))
      return;

    if (!ftypes::IsSubwayLineChecker::Instance()(ft))
      return;

    // @TODO It's necessary to cache the parsed geometry.
    ft.ParseGeometry(FeatureType::BEST_GEOMETRY);
    size_t const pointsCount = ft.GetPointsCount();
    for (size_t i = 0; i < pointsCount; ++i)
    {
      double const distMeters = MercatorBounds::DistanceOnEarth(point, ft.GetPoint(i));
      if (distMeters < closestVertexDistMeters)
      {
        closestVertexDistMeters = distMeters;
        closestVertex = SubwayVertex(
            m_numMwmIds->GetId(ft.GetID().m_mwmId.GetInfo()->GetLocalFile().GetCountryFile()),
            ft.GetID().m_index, static_cast<uint32_t >(i), SubwayType::Line);
      }
    }
  };

  m_index.ForEachInRect(
    f, MercatorBounds::RectByCenterXYAndSizeInMeters(point, kLineSearchRadiusMeters),
    scales::GetUpperScale());

  CHECK_NOT_EQUAL(closestVertexDistMeters, std::numeric_limits<double>::max(), ());
  return closestVertex;
}

bool SubwayGraph::IsValidRoad(FeatureType const & ft) const
{
  if (!m_modelFactory->GetVehicleModel()->IsRoad(ft))
    return false;

  double const speedKMPH = m_modelFactory->GetVehicleModel()->GetSpeed(ft);
  if (speedKMPH <= 0.0)
    return false;

  return true;
}
}  // namespace routing
