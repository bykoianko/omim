#pragma once

#include "coding/reader.hpp"
#include "coding/write_to_sink.hpp"

#include "std/cstdint.hpp"
#include "std/limits.hpp"

namespace routing
{
class FtPoint final
{
public:
  FtPoint() : m_featureId(0), m_pointId(0) {}

  FtPoint(uint32_t featureId, uint32_t pointId) : m_featureId(featureId), m_pointId(pointId) {}

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
}  // namespace routing
