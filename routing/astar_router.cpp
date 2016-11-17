#include "routing/astar_router.hpp"

#include "routing/base/astar_algorithm.hpp"
#include "routing/base/astar_progress.hpp"
#include "routing/bicycle_directions.hpp"
#include "routing/bicycle_model.hpp"
#include "routing/car_model.hpp"
#include "routing/features_road_graph.hpp"
#include "routing/index_graph.hpp"
#include "routing/pedestrian_model.hpp"
#include "routing/route.hpp"
#include "routing/turns_generator.hpp"

#include "indexer/feature_altitude.hpp"
#include "indexer/routing_section.hpp"

#include "geometry/distance.hpp"
#include "geometry/mercator.hpp"
#include "geometry/point2d.hpp"

#include "base/exception.hpp"

using namespace routing;

namespace
{
size_t constexpr kMaxRoadCandidates = 6;
float constexpr kProgressInterval = 2;
uint32_t constexpr kDrawPointsPeriod = 10;

vector<Junction> ConvertToJunctions(IndexGraph const & graph, vector<Joint::Id> const & joints)
{
  vector<RoadPoint> const & fsegs = graph.RedressRoute(joints);

  vector<Junction> junctions;
  junctions.reserve(fsegs.size());

  Geometry const & geometry = graph.GetGeometry();
  // TODO: Use real altitudes for pedestrian and bicycle routing.
  for (RoadPoint const & fseg : fsegs)
    junctions.emplace_back(geometry.GetPoint(fseg), feature::kDefaultAltitudeMeters);

  return junctions;
}
}  // namespace

namespace routing
{
AStarRouter::AStarRouter(const char * name, Index const & index,
                         TCountryFileFn const & countryFileFn,
                         shared_ptr<VehicleModelFactory> vehicleModelFactory,
                         shared_ptr<EdgeEstimator> estimator,
                         unique_ptr<IDirectionsEngine> directionsEngine)
  : m_name(name)
  , m_index(index)
  , m_countryFileFn(countryFileFn)
  , m_roadGraph(
        make_unique<FeaturesRoadGraph>(index, IRoadGraph::Mode::ObeyOnewayTag, vehicleModelFactory))
  , m_vehicleModelFactory(vehicleModelFactory)
  , m_estimator(estimator)
  , m_directionsEngine(move(directionsEngine))
{
  ASSERT(name, ());
  ASSERT(!m_name.empty(), ());
  ASSERT(m_vehicleModelFactory, ());
  ASSERT(m_estimator, ());
  ASSERT(m_directionsEngine, ());
}

IRouter::ResultCode AStarRouter::CalculateRoute(MwmSet::MwmId const & mwmId,
                                                m2::PointD const & startPoint,
                                                m2::PointD const & startDirection,
                                                m2::PointD const & finalPoint,
                                                RouterDelegate const & delegate, Route & route)
{
  try
  {
    return DoCalculateRoute(mwmId, startPoint, startDirection, finalPoint, delegate, route);
  }
  catch (RootException const & e)
  {
    LOG(LERROR, ("Can't find path from", MercatorBounds::ToLatLon(startPoint), "to",
                 MercatorBounds::ToLatLon(finalPoint), ":\n ", e.what()));
    return IRouter::InternalError;
  }
}

IRouter::ResultCode AStarRouter::DoCalculateRoute(MwmSet::MwmId const & mwmId,
                                                  m2::PointD const & startPoint,
                                                  m2::PointD const & /* startDirection */,
                                                  m2::PointD const & finalPoint,
                                                  RouterDelegate const & delegate, Route & route)
{
  string const country = m_countryFileFn(startPoint);

  Edge startEdge = Edge::MakeFake({} /* startJunction */, {} /* endJunction */);
  if (!FindClosestEdge(mwmId, startPoint, startEdge))
    return IRouter::StartPointNotFound;

  Edge finishEdge = Edge::MakeFake({} /* startJunction */, {} /* endJunction */);
  if (!FindClosestEdge(mwmId, finalPoint, finishEdge))
    return IRouter::EndPointNotFound;

  RoadPoint const start(startEdge.GetFeatureId().m_index, startEdge.GetSegId());
  RoadPoint const finish(finishEdge.GetFeatureId().m_index, finishEdge.GetSegId());

  IndexGraph graph(CreateGeometryLoader(m_index, mwmId,
                                        m_vehicleModelFactory->GetVehicleModelForCountry(country)),
                   m_estimator);

  if (!LoadIndex(mwmId, country, graph))
    return IRouter::RouteFileNotExist;

  AStarProgress progress(0, 100);
  progress.Initialize(graph.GetGeometry().GetPoint(start), graph.GetGeometry().GetPoint(finish));

  uint32_t drawPointsStep = 0;
  auto onVisitJunction = [&delegate, &progress, &graph, &drawPointsStep](Joint::Id const & from,
                                                                         Joint::Id const & target) {
    m2::PointD const & point = graph.GetPoint(from);
    m2::PointD const & targetPoint = graph.GetPoint(target);
    auto const lastValue = progress.GetLastValue();
    auto const newValue = progress.GetProgressForBidirectedAlgo(point, targetPoint);
    if (newValue - lastValue > kProgressInterval)
      delegate.OnProgress(newValue);
    if (drawPointsStep % kDrawPointsPeriod == 0)
      delegate.OnPointCheck(point);
    ++drawPointsStep;
  };

  AStarAlgorithm<IndexGraph> algorithm;

  RoutingResult<Joint::Id> routingResult;
  AStarAlgorithm<IndexGraph>::Result const resultCode =
      algorithm.FindPathBidirectional(graph, graph.InsertJoint(start), graph.InsertJoint(finish),
                                      routingResult, delegate, onVisitJunction);

  switch (resultCode)
  {
  case AStarAlgorithm<IndexGraph>::Result::NoPath: return IRouter::RouteNotFound;
  case AStarAlgorithm<IndexGraph>::Result::Cancelled: return IRouter::Cancelled;
  case AStarAlgorithm<IndexGraph>::Result::OK:
    vector<Junction> path = ConvertToJunctions(graph, routingResult.path);
    ReconstructRoute(m_directionsEngine.get(), *m_roadGraph, delegate, path, route);
    return IRouter::NoError;
  }
}

bool AStarRouter::FindClosestEdge(MwmSet::MwmId const & mwmId, m2::PointD const & point,
                                  Edge & closestEdge) const
{
  vector<pair<Edge, Junction>> candidates;
  m_roadGraph->FindClosestEdges(point, kMaxRoadCandidates, candidates);

  double minDistance = numeric_limits<double>::max();
  size_t minIndex = candidates.size();

  for (size_t i = 0; i < candidates.size(); ++i)
  {
    Edge const & edge = candidates[i].first;
    if (edge.GetFeatureId().m_mwmId != mwmId)
      continue;

    m2::DistanceToLineSquare<m2::PointD> squareDistance;
    squareDistance.SetBounds(edge.GetStartJunction().GetPoint(), edge.GetEndJunction().GetPoint());
    double const distance = squareDistance(point);
    if (distance < minDistance)
    {
      minDistance = distance;
      minIndex = i;
    }
  }

  if (minIndex == candidates.size())
    return false;

  closestEdge = candidates[minIndex].first;
  return true;
}

bool AStarRouter::LoadIndex(MwmSet::MwmId const & mwmId, string const & country, IndexGraph & graph)
{
  MwmSet::MwmHandle mwmHandle = m_index.GetMwmHandleById(mwmId);
  if (!mwmHandle.IsAlive())
    return false;

  MwmValue const * mwmValue = mwmHandle.GetValue<MwmValue>();
  try
  {
    my::Timer timer;
    FilesContainerR::TReader reader(mwmValue->m_cont.GetReader(ROUTING_FILE_TAG));
    ReaderSource<FilesContainerR::TReader> src(reader);
    feature::RoutingSectionHeader header;
    header.Deserialize(src);
    graph.Deserialize(src);
    LOG(LINFO,
        (ROUTING_FILE_TAG, "section for", country, "loaded in", timer.ElapsedSeconds(), "seconds"));
    return true;
  }
  catch (Reader::OpenException const & e)
  {
    LOG(LERROR, ("File", mwmValue->GetCountryFileName(), "Error while reading", ROUTING_FILE_TAG,
                 "section.", e.Msg()));
    return false;
  }
}

unique_ptr<AStarRouter> CreateCarAStarBidirectionalRouter(Index & index,
                                                          TCountryFileFn const & countryFileFn)
{
  shared_ptr<VehicleModelFactory> vehicleModelFactory = make_shared<CarModelFactory>();
  // @TODO Bicycle turn generation engine is used now. It's ok for the time being.
  // But later a special car turn generation engine should be implemented.
  unique_ptr<IDirectionsEngine> directionsEngine = make_unique<BicycleDirectionsEngine>(index);
  shared_ptr<EdgeEstimator> estimator =
      CreateCarEdgeEstimator(vehicleModelFactory->GetVehicleModel());
  unique_ptr<AStarRouter> router =
      make_unique<AStarRouter>("astar-bidirectional-car", index, countryFileFn,
                               move(vehicleModelFactory), estimator, move(directionsEngine));
  return router;
}
}  // namespace routing
