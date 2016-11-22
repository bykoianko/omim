#pragma once

#include "std/cstdint.hpp"

namespace routing
{
// RoadPoint in unique identifier for any road point in mwm file.
//
// Contains feature id and point id.
// Point id is the ordinal number of point in the road.
class RoadPoint final
{
public:
  RoadPoint() : m_featureId(0), m_pointId(0) {}

  RoadPoint(uint32_t featureId, uint32_t pointId) : m_featureId(featureId), m_pointId(pointId) {}

  uint32_t GetFeatureId() const { return m_featureId; }

  uint32_t GetPointId() const { return m_pointId; }

private:
  uint32_t m_featureId;
  uint32_t m_pointId;
};
}  // namespace routing
