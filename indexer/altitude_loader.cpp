#include "indexer/altitude_loader.hpp"

#include "base/logging.hpp"
#include "base/stl_helpers.hpp"
#include "base/thread.hpp"

#include "defines.hpp"

#include "3party/succinct/mapper.hpp"

namespace
{
void ReadBuffer(ReaderSource<FilesContainerR::TReader> & rs, vector<char> & buf)
{
  uint32_t bufSz = 0;
  rs.Read(&bufSz, sizeof(bufSz));
  if (bufSz > rs.Size() + rs.Pos())
  {
    ASSERT(false, ());
    return;
  }
  buf.clear();
  buf.resize(bufSz);
  rs.Read(buf.data(), bufSz);
}

void BitVectorInfo(succinct::rs_bit_vector const & bitVector)
{
  LOG(LINFO, ("bitVector(size, num_ones, num_zeros)", bitVector.size(), bitVector.num_ones(), bitVector.num_zeros()));
}

bool TestBitVector(succinct::rs_bit_vector const & bitVector)
{
  vector<uint32_t> const idWithAlt = {944, 26746, 8704, 945, 8759, 8704, 1541, 1544, 3080, 8744, 8747, 8748,
                                     8751, 8753, 8757, 8758, 8660, 8661, 8674, 8675, 26793, 26797, 26812, 26858};
  for (uint32_t id : idWithAlt)
  {
    if (!bitVector[id])
    {
      LOG(LINFO, ("bitVector[", id, "] = false"));
      BitVectorInfo(bitVector);
      return false;
    }
  }

  uint32_t bitnumber = 0;
  for (size_t i = 0; i < bitVector.size(); ++i)
  {
    if (bitVector[i])
      bitnumber += 1;
  }

  if (bitnumber != bitVector.rank(bitVector.size()))
  {
    LOG(LINFO, ("bitnumber =", bitnumber, "and bitVector.rank(bitVector.size() =", bitVector.rank(bitVector.size())));
    BitVectorInfo(bitVector);
    return false;
  }

  for (size_t i = 0; i < bitVector.size(); ++i)
  {
    if (bitVector.rank(i) > 24316)
    {
      LOG(LINFO, ("bitVector.rank(", i, ") > 24316"));
      BitVectorInfo(bitVector);
      return false;
    }
  }
  LOG(LINFO, ("TestBitVector passed"));
  BitVectorInfo(bitVector);
  return true;
}

//bool TestBitVector(vector<uint64_t> const & bitVector)
//{
//}

//bool TestEliasFano(succinct::elias_fano const & eliasFano){}
} // namespace

namespace feature
{
AltitudeLoader::AltitudeLoader(MwmValue const * mwmValue) : m_altitudeInfoOffset(0)
{
  LOG(LINFO, ("*****AltitudeLoader::AltitudeLoader()***** thread id =", threads::GetCurrentThreadID()));
  if (!mwmValue || mwmValue->GetHeader().GetFormat() < version::Format::v8 )
    return;

  try
  {
    FilesContainerR::TReader r = mwmValue->m_cont.GetReader(ALTITUDE_FILE_TAG);
    ReaderSource<FilesContainerR::TReader> rs(r);
    DeserializeHeader(rs);

    // Reading rs_bit_vector with altitude availability information.
    vector<char> altitudeAvailabilitBuf;
    ReadBuffer(rs, altitudeAvailabilitBuf);
//    m_altitudeAvailability = make_unique<succinct::rs_bit_vector>();
    succinct::rs_bit_vector altitudeAvailability;
    succinct::mapper::map(altitudeAvailability, altitudeAvailabilitBuf.data());
//    LOG(LINFO, ("altitudeAvailability.size() =", m_altitudeAvailability->size(),
//                "altitudeAvailability.rank(1000) =", m_altitudeAvailability->rank(1000)));
//    LOG(LINFO, ("altitudeAvailability[944 =", (*m_altitudeAvailability)[944]));
//    LOG(LINFO, ("altitudeAvailability.rank(944) =", m_altitudeAvailability->rank(944)));
    ASSERT(TestBitVector(altitudeAvailability), ());
    m_altitudeAvailability.resize(altitudeAvailability.size());
    for (size_t i = 0; i < altitudeAvailability.size(); ++i)
      m_altitudeAvailability[i] = altitudeAvailability[i];

    // Reading table with altitude ofsets for features.
//    vector<char> featureTableBuf;
//    ReadBuffer(rs, featureTableBuf);
//    m_featureTable = make_unique<succinct::elias_fano>();
//    succinct::mapper::map(*m_featureTable, featureTableBuf.data());
//    LOG(LINFO, ("m_featureTable.size() =", m_featureTable->size(), "m_featureTable.select(1000) =", m_featureTable->select(1000)));
//    LOG(LINFO, ("m_featureTable->num_ones() =", m_featureTable->num_ones()));
//    LOG(LINFO, ("m_featureTable.select(10) =", m_featureTable->select(10)));
//    LOG(LINFO, ("m_featureTable.select(10000) =", m_featureTable->select(10000)));
//    LOG(LINFO, ("m_featureTable.select(8393) =", m_featureTable->select(8393)));
//    LOG(LINFO, ("m_featureTable.select(276) =", m_featureTable->select(276)));

    //m_idx = make_unique<DDVector<TAltitudeIndexEntry, FilesContainerR::TReader>>(r);
  }
  catch (Reader::OpenException const &)
  {
    LOG(LINFO, ("MWM does not contain", ALTITUDE_FILE_TAG, "section."));
  }
}

void AltitudeLoader::DeserializeHeader(ReaderSource<FilesContainerR::TReader> & rs)
{
  TAltitudeSectionVersion version;
  rs.Read(&version, sizeof(version));
  LOG(LINFO, ("Reading version =", version));
  rs.Read(&m_minAltitude, sizeof(m_minAltitude));
  LOG(LINFO, ("Reading m_minAltitude =", m_minAltitude));
  rs.Read(&m_altitudeInfoOffset, sizeof(m_altitudeInfoOffset));
  LOG(LINFO, ("Reading m_altitudeInfoOffset =", m_altitudeInfoOffset));
}

Altitudes AltitudeLoader::GetAltitudes(uint32_t featureId) const
{
  LOG(LINFO, ("featureId =", featureId, "thread id =", threads::GetCurrentThreadID()));
  ASSERT(TestBitVector(m_altitudeAvailability), ());

//  if (!(*m_altitudeAvailability)[featureId])
//  {
//    LOG(LINFO, ("Feature featureId =", featureId, "does not contain any altitude information."));
//    return Altitudes();
//  }

//  uint64_t const r = m_altitudeAvailability->rank(featureId);
//  LOG(LINFO, ("rank =", r));
//  CHECK_LESS(r, m_altitudeAvailability->size(), (featureId));
//  uint64_t const offset = m_featureTable->select(r);
//  CHECK_LESS(offset, m_featureTable->size(), (featureId));
//  LOG(LINFO, ("offset =", offset));
//  CHECK_EQUAL(offset, r * 4, (featureId));

//  if (!m_idx || m_idx->size() == 0)
//    return Altitudes();

  // 1. Find if the feature contains any altidude information;
  // 2. If yes, find in FeaturesOffsetsTable altitude offset.
  // 2. Read altitudes for the features with offset = altitude data offset + feature offset.
//  auto it = lower_bound(m_idx->begin(), m_idx->end(),
//                        TAltitudeIndexEntry{static_cast<uint32_t>(featureId), 0, 0},
//                        my::LessBy(&TAltitudeIndexEntry::featureId));

//  if (it == m_idx->end())
//    return Altitudes();

//  if (featureId != it->featureId)
//  {
//    ASSERT(false, ());
//    return Altitudes();
//  }

//  if (it->beginAlt == kInvalidAltitude || it->endAlt == kInvalidAltitude)
//    return Altitudes();

//  return Altitudes(it->beginAlt, it->endAlt);
  return Altitudes();
}
} // namespace feature
