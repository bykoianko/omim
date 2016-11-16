#pragma once

#include "coding/reader.hpp"
#include "coding/write_to_sink.hpp"

#include "std/cstdint.hpp"
#include "std/limits.hpp"
#include "std/sstream.hpp"
#include "std/string.hpp"

namespace routing
{
class RoadPoint final
{
public:
  RoadPoint() : m_featureId(0), m_pointId(0) {}

  RoadPoint(uint32_t featureId, uint32_t pointId) : m_featureId(featureId), m_pointId(pointId) {}

  bool operator==(RoadPoint const & roadPoint) const
  {
    return m_featureId == roadPoint.m_featureId && m_pointId == roadPoint.m_pointId;
  }

  uint32_t GetFeatureId() const { return m_featureId; }

  uint32_t GetPointId() const { return m_pointId; }

  template <class Sink>
  void Serialize(Sink & sink) const
  {
    WriteToSink(sink, m_featureId);
    WriteToSink(sink, m_pointId);
  }

  template <class Source>
  void Deserialize(Source & src)
  {
    m_featureId = ReadPrimitiveFromSource<decltype(m_featureId)>(src);
    m_pointId = ReadPrimitiveFromSource<decltype(m_pointId)>(src);
  }

private:
  uint32_t m_featureId;
  uint32_t m_pointId;
};

inline string DebugPrint(RoadPoint const & roadPoint)
{
  ostringstream out;
  out << "RoadPoint[" << roadPoint.GetFeatureId() << ", " << roadPoint.GetPointId() << "]";
  return out.str();
}
}  // namespace routing
