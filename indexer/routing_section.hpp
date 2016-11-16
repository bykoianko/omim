#pragma once

#include "coding/reader.hpp"
#include "coding/write_to_sink.hpp"

#include "std/cstdint.hpp"

namespace feature
{
class RoutingSectionHeader final
{
public:
  RoutingSectionHeader() : m_version(0) {}

  uint8_t GetVersion() const { return m_version; }

  template <class TSink>
  void Serialize(TSink & sink) const
  {
    WriteToSink(sink, m_version);
  }

  template <class TSource>
  void Deserialize(TSource & src)
  {
    m_version = ReadPrimitiveFromSource<decltype(m_version)>(src);
  }

private:
  uint8_t m_version;
};
}  // namespace feature
