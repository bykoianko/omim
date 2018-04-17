#include "routing/index_graph_loader.hpp"

#include "routing/index_graph_serialization.hpp"
#include "routing/restriction_loader.hpp"
#include "routing/road_access_serialization.hpp"
#include "routing/routing_exceptions.hpp"

#include "coding/file_container.hpp"

#include "base/assert.hpp"
#include "base/timer.hpp"

namespace
{
using namespace routing;
using namespace std;

class IndexGraphLoaderImpl final : public IndexGraphLoader
{
public:
  IndexGraphLoaderImpl(VehicleType vehicleType, bool loadAltitudes, shared_ptr<NumMwmIds> numMwmIds,
                       shared_ptr<VehicleModelFactoryInterface> vehicleModelFactory,
                       shared_ptr<EdgeEstimator> estimator, Index & index);
  
  ~IndexGraphLoaderImpl()
  {
    return;
  }

  // IndexGraphLoader overrides:
  Geometry & GetGeometry(NumMwmId numMwmId) override;
  IndexGraph & GetIndexGraph(NumMwmId numMwmId) override;
  void Clear() override;
  size_t GetSize() const override;

private:
  IndexGraph & Init(NumMwmId mwmId);
  IndexGraph & Deserialize(NumMwmId numMwmId, IndexGraph & graph);

  VehicleType m_vehicleType;
  bool m_loadAltitudes;
  Index & m_index;
  shared_ptr<NumMwmIds> m_numMwmIds;
  shared_ptr<VehicleModelFactoryInterface> m_vehicleModelFactory;
  shared_ptr<EdgeEstimator> m_estimator;
  unordered_map<NumMwmId, unique_ptr<IndexGraph>> m_graphs;
};

IndexGraphLoaderImpl::IndexGraphLoaderImpl(
    VehicleType vehicleType, bool loadAltitudes, shared_ptr<NumMwmIds> numMwmIds,
    shared_ptr<VehicleModelFactoryInterface> vehicleModelFactory,
    shared_ptr<EdgeEstimator> estimator, Index & index)
  : m_vehicleType(vehicleType)
  , m_loadAltitudes(loadAltitudes)
  , m_index(index)
  , m_numMwmIds(numMwmIds)
  , m_vehicleModelFactory(vehicleModelFactory)
  , m_estimator(estimator)
{
  CHECK(m_numMwmIds, ());
  CHECK(m_vehicleModelFactory, ());
  CHECK(m_estimator, ());
}

//IndexGraph & IndexGraphLoaderImpl::GetIndexGraph(NumMwmId numMwmId)
//{
//  auto it = m_graphs.find(numMwmId);
//  if (it != m_graphs.end())
//    return *it->second;
//
//  return Init(numMwmId);
//}
//
//Geometry & IndexGraphLoaderImpl::GetGeometry(NumMwmId numMwmId)
//{
//  return GetIndexGraph(numMwmId).GetGeometry();
//}
//
//IndexGraph & IndexGraphLoaderImpl::Init(NumMwmId numMwmId)
//{
//  platform::CountryFile const & file = m_numMwmIds->GetFile(numMwmId);
//  MwmSet::MwmHandle handle = m_index.GetMwmHandleByCountryFile(file);
//  if (!handle.IsAlive())
//    MYTHROW(RoutingException, ("Can't get mwm handle for", file));
//
//  shared_ptr<VehicleModelInterface> vehicleModel =
//      m_vehicleModelFactory->GetVehicleModelForCountry(file.GetName());
//
//  auto graphPtr = make_unique<IndexGraph>(
//      GeometryLoader::Create(m_index, handle, vehicleModel, m_loadAltitudes),
//      m_estimator);
//  IndexGraph & graph = *graphPtr;
//
//  my::Timer timer;
//  MwmValue const & mwmValue = *handle.GetValue<MwmValue>();
//  DeserializeIndexGraph(mwmValue, m_vehicleType, graph);
//  m_graphs[numMwmId] = move(graphPtr);
//  LOG(LINFO, (ROUTING_FILE_TAG, "section for", file.GetName(), "loaded in", timer.ElapsedSeconds(),
//      "seconds"));
//  return graph;
//}

Geometry & IndexGraphLoaderImpl::GetGeometry(NumMwmId numMwmId)
{
  auto it = m_graphs.find(numMwmId);
  if (it != m_graphs.end())
    return it->second->GetGeometry();

  return Init(numMwmId).GetGeometry();
}

IndexGraph & IndexGraphLoaderImpl::GetIndexGraph(NumMwmId numMwmId)
{
  auto it = m_graphs.find(numMwmId);
  // @TODO graph is found but it'll find one more time in Deserialize(numMwmId).
  // @TODO Build and Deserialize means the same.
  if (it != m_graphs.end())
    return it->second->IsBuilt() ? *it->second : Deserialize(numMwmId, *it->second);

  return Deserialize(numMwmId, Init(numMwmId));
}

IndexGraph & IndexGraphLoaderImpl::Init(NumMwmId numMwmId)
{
  platform::CountryFile const & file = m_numMwmIds->GetFile(numMwmId);
  MwmSet::MwmHandle handle = m_index.GetMwmHandleByCountryFile(file);
  if (!handle.IsAlive())
    MYTHROW(RoutingException, ("Can't get mwm handle for", file));

  shared_ptr<VehicleModelInterface> vehicleModel =
      m_vehicleModelFactory->GetVehicleModelForCountry(file.GetName());

  return *(
      m_graphs[numMwmId] = make_unique<IndexGraph>(
          GeometryLoader::Create(m_index, handle, vehicleModel, m_loadAltitudes), m_estimator));
}

IndexGraph & IndexGraphLoaderImpl::Deserialize(NumMwmId numMwmId, IndexGraph & graph)
{
  CHECK(!graph.IsBuilt(), ());
  platform::CountryFile const & file = m_numMwmIds->GetFile(numMwmId);
  MwmSet::MwmHandle handle = m_index.GetMwmHandleByCountryFile(file);
  if (!handle.IsAlive())
    MYTHROW(RoutingException, ("Can't get mwm handle for", file));

  my::Timer timer;
  MwmValue const & mwmValue = *handle.GetValue<MwmValue>();
  DeserializeIndexGraph(mwmValue, m_vehicleType, graph);
  LOG(LINFO, (ROUTING_FILE_TAG, "section for", file.GetName(), "loaded in", timer
      .ElapsedSeconds(), "seconds"));
  return graph;
}

void IndexGraphLoaderImpl::Clear() { m_graphs.clear(); }

size_t IndexGraphLoaderImpl::GetSize() const
{
  size_t sz = 0;
  for (auto const & kv : m_graphs)
    sz += (4 + kv.second->GetSize());
  return sz;
}

bool ReadRoadAccessFromMwm(MwmValue const & mwmValue, VehicleType vehicleType,
                           RoadAccess & roadAccess)
{
  if (!mwmValue.m_cont.IsExist(ROAD_ACCESS_FILE_TAG))
    return false;

  try
  {
    auto const reader = mwmValue.m_cont.GetReader(ROAD_ACCESS_FILE_TAG);
    ReaderSource<FilesContainerR::TReader> src(reader);

    RoadAccessSerializer::Deserialize(src, vehicleType, roadAccess);
  }
  catch (Reader::OpenException const & e)
  {
    LOG(LERROR, ("Error while reading", ROAD_ACCESS_FILE_TAG, "section.", e.Msg()));
    return false;
  }
  return true;
}
}  // namespace

namespace routing
{
// static
unique_ptr<IndexGraphLoader> IndexGraphLoader::Create(
    VehicleType vehicleType, bool loadAltitudes, shared_ptr<NumMwmIds> numMwmIds,
    shared_ptr<VehicleModelFactoryInterface> vehicleModelFactory, shared_ptr<EdgeEstimator> estimator,
    Index & index)
{
  return make_unique<IndexGraphLoaderImpl>(vehicleType, loadAltitudes, numMwmIds, vehicleModelFactory,
                                           estimator, index);
}

void DeserializeIndexGraph(MwmValue const & mwmValue, VehicleType vehicleType, IndexGraph & graph)
{
  FilesContainerR::TReader reader(mwmValue.m_cont.GetReader(ROUTING_FILE_TAG));
  ReaderSource<FilesContainerR::TReader> src(reader);
  IndexGraphSerializer::Deserialize(graph, src, GetVehicleMask(vehicleType));
  RestrictionLoader restrictionLoader(mwmValue, graph);
  if (restrictionLoader.HasRestrictions())
    graph.SetRestrictions(restrictionLoader.StealRestrictions());

  RoadAccess roadAccess;
  if (ReadRoadAccessFromMwm(mwmValue, vehicleType, roadAccess))
    graph.SetRoadAccess(move(roadAccess));
}
}  // namespace routing
