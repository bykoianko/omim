#pragma once

#include "coding/reader.hpp"
#include "coding/write_to_sink.hpp"

#include "std/cstdint.hpp"
#include "std/limits.hpp"

namespace routing
{
class FSegId final
{
public:
  FSegId() : m_featureId(0), m_segId(0) {}

  FSegId(uint32_t featureId, uint32_t segId) : m_featureId(featureId), m_segId(segId) {}

  uint32_t GetFeatureId() const { return m_featureId; }

  uint32_t GetSegId() const { return m_segId; }

  template <class Sink>
  void Serialize(Sink & sink) const
  {
    WriteToSink(sink, m_featureId);
    WriteToSink(sink, m_segId);
  }

  template <class Source>
  void Deserialize(Source & src)
  {
    m_featureId = ReadPrimitiveFromSource<decltype(m_featureId)>(src);
    m_segId = ReadPrimitiveFromSource<decltype(m_segId)>(src);
  }

private:
  uint32_t m_featureId;
  uint32_t m_segId;
};
}  // namespace routing
