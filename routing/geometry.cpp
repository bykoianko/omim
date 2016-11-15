#include "geometry.hpp"

#include "geometry/mercator.hpp"

#include "base/assert.hpp"

using namespace routing;

namespace
{
class GeometryLoaderImpl final : public GeometryLoader
{
public:
  GeometryLoaderImpl(Index const & index, MwmSet::MwmId const & mwmId,
                     shared_ptr<IVehicleModel> vehicleModel);

  // Geometry overrides:
  virtual void Load(uint32_t featureId, RoadGeometry & road) const override;

private:
  Index const & m_index;
  MwmSet::MwmId const m_mwmId;
  shared_ptr<IVehicleModel> m_vehicleMode;
};

GeometryLoaderImpl::GeometryLoaderImpl(Index const & index, MwmSet::MwmId const & mwmId,
                                       shared_ptr<IVehicleModel> vehicleModel)
  : m_index(index), m_mwmId(mwmId), m_vehicleMode(vehicleModel)
{
  ASSERT(m_vehicleMode, ());
}

void GeometryLoaderImpl::Load(uint32_t featureId, RoadGeometry & road) const
{
  Index::FeaturesLoaderGuard guard(m_index, m_mwmId);
  FeatureType feature;
  bool const isFound = guard.GetFeatureByIndex(featureId, feature);
  ASSERT(isFound, ("Feature", featureId, "not found"));
  if (!isFound)
    return;

  feature.ParseGeometry(FeatureType::BEST_GEOMETRY);
  road.Load(*m_vehicleMode, feature);
}
}  // namespace

namespace routing
{
RoadGeometry::RoadGeometry(bool oneWay, double speed, buffer_vector<m2::PointD, 32> const & points)
  : m_isRoad(true), m_isOneWay(oneWay), m_speed(speed), m_points(points)
{
}

void RoadGeometry::Load(IVehicleModel const & vehicleModel, FeatureType & feature)
{
  m_isRoad = vehicleModel.IsRoad(feature);
  m_isOneWay = vehicleModel.IsOneWay(feature);
  m_speed = vehicleModel.GetSpeed(feature);
  feature.SwapPoints(m_points);
}

Geometry::Geometry(unique_ptr<GeometryLoader> loader) : m_loader(move(loader))
{
  ASSERT(m_loader, ());
}

unique_ptr<GeometryLoader> CreateGeometryLoader(Index const & index, MwmSet::MwmId const & mwmId,
                                                shared_ptr<IVehicleModel> vehicleModel)
{
  return make_unique<GeometryLoaderImpl>(index, mwmId, vehicleModel);
}
}  // namespace routing
