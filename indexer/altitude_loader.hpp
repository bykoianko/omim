#pragma once
#include "indexer/feature_altitude.hpp"
#include "indexer/index.hpp"

#include "coding/dd_vector.hpp"

#include "3party/succinct/rs_bit_vector.hpp"

#include "std/unique_ptr.hpp"

namespace feature
{
using TAltitudeSectionVersion = uint16_t;
using TAltitudeSectionOffset = uint32_t;

class AltitudeLoader
{
public:
  AltitudeLoader(MwmValue const * mwmValue);
//  ~AltitudeLoader()
//  {
//    LOG(LINFO, ("~AltitudeLoader()"));
//  }

  Altitudes GetAltitudes(uint32_t featureId) const;

private:
  void DeserializeHeader(ReaderSource<FilesContainerR::TReader> & rs);

//  unique_ptr<succinct::rs_bit_vector> m_altitudeAvailability;
  vector<uint64_t> m_altitudeAvailability;
//  succinct::rs_bit_vector m_altitudeAvailability;
//  unique_ptr<succinct::elias_fano> m_featureTable;
  TAltitudeSectionOffset m_altitudeInfoOffset;
  TAltitude m_minAltitude;
};
} // namespace feature
