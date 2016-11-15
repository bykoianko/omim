#pragma once

#include "routing/fseg.hpp"
#include "routing/vehicle_model.hpp"

#include "indexer/index.hpp"

#include "geometry/point2d.hpp"

#include "base/buffer_vector.hpp"

#include "std/cstdint.hpp"
#include "std/shared_ptr.hpp"
#include "std/unique_ptr.hpp"

namespace routing
{
class RoadGeometry final
{
public:
  RoadGeometry() = default;
  RoadGeometry(bool oneWay, double speed, buffer_vector<m2::PointD, 32> const & points);

  void Load(IVehicleModel const & vehicleModel, FeatureType & feature);

  bool IsRoad() const { return m_isRoad; }

  bool IsOneWay() const { return m_isOneWay; }

  bool GetSpeed() const { return m_speed; }

  m2::PointD const & GetPoint(uint32_t segId) const
  {
    ASSERT_LESS(segId, m_points.size(), ());
    return m_points[segId];
  }

  uint32_t GetPointsCount() const { return m_points.size(); }

private:
  bool m_isRoad = false;
  bool m_isOneWay = false;
  double m_speed = 0.0;
  buffer_vector<m2::PointD, 32> m_points;
};

class GeometryLoader
{
public:
  virtual ~GeometryLoader() = default;

  virtual void Load(uint32_t featureId, RoadGeometry & road) const = 0;
};

class Geometry final
{
public:
  Geometry() = default;
  explicit Geometry(unique_ptr<GeometryLoader> loader);

  RoadGeometry const & GetRoad(uint32_t featureId) const
  {
    auto const & it = m_roads.find(featureId);
    if (it != m_roads.cend())
      return it->second;

    RoadGeometry & road = m_roads[featureId];
    m_loader->Load(featureId, road);
    return road;
  }

  m2::PointD const & GetPoint(FSegId const & fseg) const
  {
    return GetRoad(fseg.GetFeatureId()).GetPoint(fseg.GetSegId());
  }

private:
  // Feature id to RoadGeometry map.
  mutable unordered_map<uint32_t, RoadGeometry> m_roads;
  unique_ptr<GeometryLoader> m_loader;
};

unique_ptr<GeometryLoader> CreateGeometryLoader(Index const & index, MwmSet::MwmId const & mwmId,
                                                shared_ptr<IVehicleModel> vehicleModel);
}  // namespace routing
