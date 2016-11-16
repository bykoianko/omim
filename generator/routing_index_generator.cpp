#include "generator/routing_index_generator.hpp"

#include "routing/bicycle_model.hpp"
#include "routing/car_model.hpp"
#include "routing/index_graph.hpp"
#include "routing/pedestrian_model.hpp"
#include "routing/vehicle_model.hpp"

#include "indexer/feature.hpp"
#include "indexer/feature_processor.hpp"
#include "indexer/index.hpp"
#include "indexer/routing_section.hpp"
#include "indexer/scales.hpp"

#include "platform/country_file.hpp"
#include "platform/local_country_file.hpp"

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
uint32_t constexpr kFixPointFactor = 100000;

inline m2::PointI PointDToPointI(m2::PointD const & p) { return m2::PointI(p * kFixPointFactor); }

uint64_t CalcLocationKey(m2::PointD const & point)
{
  m2::PointI const pointI(PointDToPointI(point));
  return (static_cast<uint64_t>(pointI.y) << 32) + static_cast<uint64_t>(pointI.x);
}

class Processor final
{
public:
  Processor(string const & dir, string const & country)
    : m_pedestrianModel(make_unique<PedestrianModelFactory>()->GetVehicleModelForCountry(country))
    , m_bicycleModel(make_unique<BicycleModelFactory>()->GetVehicleModelForCountry(country))
    , m_carModel(make_unique<CarModelFactory>()->GetVehicleModelForCountry(country))
  {
    LocalCountryFile localCountryFile(dir, CountryFile(country), 1 /* version */);
    m_index.RegisterMap(localCountryFile);
    vector<shared_ptr<MwmInfo>> info;
    m_index.GetMwmsInfo(info);
    CHECK_EQUAL(info.size(), 1, ());
    CHECK(info[0], ());
  }

  void operator()(FeatureType const & f)
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
      uint64_t const locationKey = CalcLocationKey(f.GetPoint(fromSegId));
      m_posToJoint[locationKey].AddEntry(FSegId(id, fromSegId));
    }
  }

  void ProcessAllFeatures() { m_index.ForEachInScale(*this, scales::GetUpperScale()); }

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

    graph.Export(joints);
  }

private:
  bool IsRoad(FeatureType const & f) const
  {
    return m_pedestrianModel->IsRoad(f) || m_bicycleModel->IsRoad(f) || m_carModel->IsRoad(f);
  }

  Index m_index;
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
    Processor processor(dir, country);
    string const datFile = my::JoinFoldersToPath(dir, country + DATA_FILE_EXTENSION);
    LOG(LINFO, ("datFile =", datFile));
    processor.ProcessAllFeatures();
    processor.RemoveNonConnections();

    IndexGraph graph;
    processor.BuildGraph(graph);
    LOG(LINFO, ("Routing index contains", graph.GetRoadsAmount(), "roads", graph.GetJointsAmount(),
                "joints", graph.GetFSegsAmount(), "fsegs"));

    FilesContainerW cont(datFile, FileWriter::OP_WRITE_EXISTING);
    FileWriter writer = cont.GetWriter(ROUTING_FILE_TAG);

    RoutingSectionHeader const header;
    header.Serialize(writer);
    graph.Serialize(writer);
  }
  catch (RootException const & e)
  {
    LOG(LERROR, ("An exception happened while creating", ROUTING_FILE_TAG, "section:", e.what()));
  }
}
}  // namespace routing
