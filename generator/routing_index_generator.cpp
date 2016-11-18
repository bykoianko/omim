#include "generator/routing_index_generator.hpp"

#include "routing/bicycle_model.hpp"
#include "routing/car_model.hpp"
#include "routing/index_graph.hpp"
#include "routing/pedestrian_model.hpp"
#include "routing/vehicle_model.hpp"

#include "indexer/feature.hpp"
#include "indexer/features_vector.hpp"
#include "indexer/point_to_int64.hpp"
#include "indexer/routing_section.hpp"

#include "platform/country_file.hpp"
#include "platform/local_country_file.hpp"
#include "platform/mwm_traits.hpp"

#include "coding/file_container.hpp"
#include "coding/file_name_utils.hpp"

#include "base/logging.hpp"

#include "std/shared_ptr.hpp"
#include "std/unique_ptr.hpp"
#include "std/unordered_map.hpp"
#include "std/vector.hpp"

using namespace feature;
using namespace platform;
using namespace routing;

namespace
{
class Processor final
{
public:
  Processor(string const & country)
    : m_pedestrianModel(make_unique<PedestrianModelFactory>()->GetVehicleModelForCountry(country))
    , m_bicycleModel(make_unique<BicycleModelFactory>()->GetVehicleModelForCountry(country))
    , m_carModel(make_unique<CarModelFactory>()->GetVehicleModelForCountry(country))
  {
  }

  void ProcessAllFeatures(FeaturesVector const & features)
  {
    features.ForEach(
        [this](FeatureType & feature, uint32_t /* featureId */) { ProcessFeature(feature); });
  }

  void RemoveNonConnections()
  {
    for (auto it = m_posToJoint.begin(); it != m_posToJoint.end();)
    {
      if (it->second.GetSize() < 2)
        it = m_posToJoint.erase(it);
      else
        ++it;
    }
  }

  void BuildGraph(IndexGraph & graph) const
  {
    vector<Joint> joints;
    joints.reserve(m_posToJoint.size());
    for (auto const & it : m_posToJoint)
      joints.emplace_back(it.second);

    graph.Import(joints);
  }

private:
  bool IsRoad(FeatureType const & f) const
  {
    return m_pedestrianModel->IsRoad(f) || m_bicycleModel->IsRoad(f) || m_carModel->IsRoad(f);
  }

  void ProcessFeature(FeatureType const & f)
  {
    if (!IsRoad(f))
      return;

    uint32_t const id = f.GetID().m_index;
    f.ParseGeometry(FeatureType::BEST_GEOMETRY);
    size_t const pointsCount = f.GetPointsCount();
    if (pointsCount == 0)
      return;

    for (size_t fromSegId = 0; fromSegId < pointsCount; ++fromSegId)
    {
      uint64_t const locationKey = PointToInt64(f.GetPoint(fromSegId), POINT_COORD_BITS);
      m_posToJoint[locationKey].AddPoint(RoadPoint(id, fromSegId));
    }
  }

  shared_ptr<IVehicleModel> m_pedestrianModel;
  shared_ptr<IVehicleModel> m_bicycleModel;
  shared_ptr<IVehicleModel> m_carModel;
  unordered_map<uint64_t, Joint> m_posToJoint;
};
}  // namespace

namespace routing
{
void BuildRoutingIndex(string const & dir, string const & country)
{
  LOG(LINFO, ("Building routing index for", country, "in directory", dir));
  try
  {
    string const filename = my::JoinFoldersToPath(dir, country + DATA_FILE_EXTENSION);

    FilesContainerR rcont(filename);
    feature::DataHeader header(rcont);

    version::MwmTraits const traits(header.GetFormat());
    if (!traits.HasOffsetsTable())
    {
      LOG(LERROR, (filename, "does not have an offsets table!"));
      return;
    }

    auto const table = feature::FeaturesOffsetsTable::Load(rcont);
    if (!table)
    {
      LOG(LERROR, ("Can't load offsets table from:", filename));
      return;
    }

    FeaturesVector const features(rcont, header, table.get());

    Processor processor(country);
    processor.ProcessAllFeatures(features);
    processor.RemoveNonConnections();

    IndexGraph graph;
    processor.BuildGraph(graph);
    LOG(LINFO, ("Routing index contains", graph.GetNumRoads(), "roads", graph.GetNumJoints(),
                "joints", graph.GetNumPoints(), "points"));

    FilesContainerW cont(filename, FileWriter::OP_WRITE_EXISTING);
    FileWriter writer = cont.GetWriter(ROUTING_FILE_TAG);

    RoutingSectionHeader const routingHeader;
    routingHeader.Serialize(writer);
    graph.Serialize(writer);
  }
  catch (RootException const & e)
  {
    LOG(LERROR, ("An exception happened while creating", ROUTING_FILE_TAG, "section:", e.what()));
  }
}
}  // namespace routing
