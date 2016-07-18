#pragma once

#include "coding/varint.hpp"

#include "std/limits.hpp"
#include "std/vector.hpp"

namespace feature
{
using TAltitude = int16_t;
using TAltitudes = vector<feature::TAltitude>;
TAltitude constexpr kInvalidAltitude = numeric_limits<TAltitude>::min();

//struct TAltitudeIndexEntry
//{
//  uint32_t featureId;
//  feature::TAltitude beginAlt;
//  feature::TAltitude endAlt;
//};

struct Altitudes
{
  Altitudes() = default;
  Altitudes(TAltitude b, TAltitude e) : begin(b), end(e) {}

  TAltitude begin = kInvalidAltitude;
  TAltitude end = kInvalidAltitude;
};

class Altitude
{
public:
  Altitude() = default;
  Altitude(/*uint32_t featureId, */Altitudes const & altitudes)
    : /*m_featureId(featureId),*/ m_altitudes(altitudes)
  {
  }

  template <class TSink>
  void Serialize(TSink & sink) const
  {
//    sink.Write(&m_featureId, sizeof(uint32_t));
    sink.Write(&m_altitudes.begin, sizeof(TAltitude));
    sink.Write(&m_altitudes.end, sizeof(TAltitude));
  }

  /// @TODO template <class TSource> void Deserialize(TSource & src) should be implement here.
  /// But now for test purposes deserialization is done with DDVector construction.
  template <class TSource>
  void Deserialize(TSource & src)
  {
    src.Read(&m_altitudes.begin, sizeof(TAltitude));
    src.Read(&m_altitudes.end, sizeof(TAltitude));
  }

//  uint32_t GetFeatureId() const { return m_featureId; }
//  Altitudes const & GetAltitudes() const { return m_altitudes; }

private:
  /// @TODO Note. Feature id is located here because there's no index for altitudes.
  /// There's only pairs sorted by feature id. Before merging to master some index has to be
  /// implemented.
  /// Don't forget to remove |m_featureId|.
  //uint32_t m_featureId = 0;
  Altitudes m_altitudes;
};
// @TODO Add functions for writing and reading header of sectiona and index here.
}  // namespace feature
