#include "generator/routing_generator.hpp"
#include "generator/srtm_parser.hpp"

#include "routing/routing_helpers.hpp"

#include "indexer/altitude_loader.hpp"
#include "indexer/feature.hpp"
#include "indexer/feature_altitude.hpp"
#include "indexer/feature_data.hpp"
#include "indexer/feature_processor.hpp"

#include "coding/file_container.hpp"
#include "coding/file_name_utils.hpp"
#include "coding/read_write_utils.hpp"
#include "coding/reader.hpp"
#include "coding/varint.hpp"

#include "coding/internal/file_data.hpp"

#include "base/assert.hpp"
#include "base/logging.hpp"
#include "base/stl_helpers.hpp"
#include "base/string_utils.hpp"

#include "defines.hpp"

#include "3party/succinct/elias_fano.hpp"
#include "3party/succinct/mapper.hpp"
#include "3party/succinct/rs_bit_vector.hpp"

#include "std/algorithm.hpp"
#include "std/type_traits.hpp"
#include "std/utility.hpp"
#include "std/vector.hpp"

using namespace feature;

namespace
{
class Processor
{
public:
  using TFeatureAltitude = pair<uint32_t, Altitudes>;
  using TFeatureAltitudes = vector<TFeatureAltitude>;

  Processor(string const & srtmPath) : m_srtmManager(srtmPath), m_minAltitude(kInvalidAltitude) {}

  TFeatureAltitudes const & GetFeatureAltitudes() const { return m_featureAltitudes; }

  vector<bool> const & GetAltitudeAvailability() const { return m_altitudeAvailability; }

  TAltitude GetMinAltitude() const { return m_minAltitude; }

  void operator()(FeatureType const & f, uint32_t const & id)
  {
    if (id != m_altitudeAvailability.size())
    {
      LOG(LERROR, ("There's a gap in feature id order."));
      return;
    }

    bool hasAltitude = false;
    do
    {
      if (!routing::IsRoad(feature::TypesHolder(f)))
        break;

      f.ParseGeometry(FeatureType::BEST_GEOMETRY);
      size_t const pointsCount = f.GetPointsCount();
      if (pointsCount == 0)
        break;

      TAltitude const beginningAlt = m_srtmManager.GetHeight(MercatorBounds::ToLatLon(f.GetPoint(0)));
      TAltitude const endAlt = m_srtmManager.GetHeight(MercatorBounds::ToLatLon(f.GetPoint(pointsCount - 1)));
      if (beginningAlt == kInvalidAltitude || endAlt == kInvalidAltitude)
        break;
      hasAltitude = true;
      m_featureAltitudes.push_back(make_pair(id, Altitudes(beginningAlt, endAlt)));

      // @TODO Checks all feature points to calculate minFeatureAltitude.
      TAltitude const minFeatureAltitude = min(beginningAlt, endAlt);
      if (m_minAltitude == kInvalidAltitude)
        m_minAltitude = minFeatureAltitude;
      else
        m_minAltitude = min(minFeatureAltitude, m_minAltitude);
    }
    while(false);

    m_altitudeAvailability.push_back(hasAltitude);

    vector<uint32_t> const checketId = {26746, 8704, 945, 8759, 8704, 1541, 1544, 3080, 8744, 8747, 8748, 8751, 8753, 8757, 8758};
    if (find(checketId.begin(), checketId.end(), id) != checketId.end())
      LOG(LINFO, ("hasAltitude =", hasAltitude));
  }

  bool HasAltitudeInfo()
  {
    return !m_featureAltitudes.empty();
  }

  void SortFeatureAltitudes()
  {
    sort(m_featureAltitudes.begin(), m_featureAltitudes.end(), my::LessBy(&Processor::TFeatureAltitude::first));
  }

private:
  generator::SrtmTileManager m_srtmManager;
  TFeatureAltitudes m_featureAltitudes;
  vector<bool> m_altitudeAvailability;
  TAltitude m_minAltitude;
};

uint32_t GetFileSize(string const & filePath)
{
  uint64_t size;
  if (!my::GetFileSize(filePath, size))
  {
    LOG(LWARNING, ("altitudeAvailability", filePath, "size = 0"));
    return 0;
  }

  LOG(LINFO, ("altitudeAvailability ", filePath, "size =", size, "bytes"));
  return size;
}

void MoveFileToAltitudeSection(string const & filePath, uint32_t fileSize, FileWriter & w)
{
  {
    ReaderSource<FileReader> r = FileReader(filePath);
    w.Write(&fileSize, sizeof(fileSize));
    rw::ReadAndWrite(r, w);
    LOG(LINFO, ("After writing bit vector w.Pos() =", w.Pos(), "r.Pos() from", filePath, "=", r.Pos()));
  }
  FileWriter::DeleteFileX(filePath);
}

void SerializeHeader(TAltitudeSectionVersion version, TAltitude minAltitude,
                     TAltitudeSectionOffset altitudeInfoOffset, FileWriter & w)
{
  w.Write(&version, sizeof(version));

  w.Write(&minAltitude, sizeof(minAltitude));
  LOG(LINFO, ("altitudeAvailability writing minAltitude =", minAltitude));

  w.Write(&altitudeInfoOffset, sizeof(altitudeInfoOffset));
  LOG(LINFO, ("altitudeAvailability writing altitudeInfoOffset =", altitudeInfoOffset));
}
}  // namespace

namespace routing
{
void BuildRoadAltitudes(string const & srtmPath, string const & baseDir, string const & countryName)
{
  try
  {
    // Preparing altitude information.
    LOG(LINFO, ("srtmPath =", srtmPath, "baseDir =", baseDir, "countryName =", countryName));
    string const mwmPath = my::JoinFoldersToPath(baseDir, countryName + DATA_FILE_EXTENSION);

    Processor processor(srtmPath);
    feature::ForEachFromDat(mwmPath, processor);
    processor.SortFeatureAltitudes();
    Processor::TFeatureAltitudes const & featureAltitudes = processor.GetFeatureAltitudes();

    if (!processor.HasAltitudeInfo())
    {
      LOG(LINFO, ("No altitude information for road features of mwm", countryName));
      return;
    }

    // Writing compressed bit vector with features which have altitude information.
    succinct::rs_bit_vector altitudeAvailability(processor.GetAltitudeAvailability());
    string const altitudeAvailabilityPath = my::JoinFoldersToPath(baseDir, "altitude_availability.bitvector");
    LOG(LINFO, ("altitudeAvailability succinct::mapper::size_of(altitudeAvailability) =", succinct::mapper::size_of(altitudeAvailability)));
    succinct::mapper::freeze(altitudeAvailability, altitudeAvailabilityPath.c_str());
    LOG(LINFO, ("altitudeAvailability.size() =", altitudeAvailability.size(),
                "altitudeAvailability.rank(1000) =", altitudeAvailability.rank(1000)));

    // Writing feature altitude information to a file and memorizing the offsets.
    string const altitudeInfoPath = my::JoinFoldersToPath(baseDir, "altitude_info");
    vector<uint32_t> offsets;
    offsets.reserve(featureAltitudes.size());
    {
      FileWriter altitudeInfoW(altitudeInfoPath);
      for (auto const & a : featureAltitudes)
      {
        offsets.push_back(altitudeInfoW.Pos());
        Altitude altitude(a.second /* feature altitudes */);
        altitude.Serialize(altitudeInfoW);
      }
    }
    LOG(LINFO, ("Altitude was written for", featureAltitudes.size(), "features."));

    // Writing feature altitude offsets.
    CHECK(is_sorted(offsets.begin(), offsets.end()), ());
    CHECK(adjacent_find(offsets.begin(), offsets.end()) == offsets.end(), ());
    LOG(LINFO, ("Max altitude info offset =", offsets.back(), "offsets.size() =", offsets.size()));
    succinct::elias_fano::elias_fano_builder builder(offsets.back(), offsets.size());
    for (uint32_t offset : offsets)
      builder.push_back(offset);

    succinct::elias_fano featureTable(&builder);
    LOG(LINFO, ("featureTable.size() =", featureTable.size(), "featureTable.select(1000) =", featureTable.select(1000)));
    LOG(LINFO, ("featureTable.select(10) =", featureTable.select(10)));
    LOG(LINFO, ("featureTable.select(10000) =", featureTable.select(10000)));
    string const featuresTablePath = my::JoinFoldersToPath(baseDir, "altitude_offsets.elias_fano");
    succinct::mapper::freeze(featureTable, featuresTablePath.c_str());

    uint32_t altitudeAvailabilitySize = GetFileSize(altitudeAvailabilityPath);
    // @TODO Remove section altitudeInfoSize.
    uint32_t altitudeInfoSize = GetFileSize(altitudeInfoPath);
    uint32_t featuresTableSize = GetFileSize(featuresTablePath);

    FilesContainerW cont(mwmPath, FileWriter::OP_WRITE_EXISTING);
    FileWriter w = cont.GetWriter(ALTITUDE_FILE_TAG);

    // Writing section with altitude information.
    // Writing altitude section header.
    TAltitudeSectionVersion const version = 1;
    TAltitude const minAltitude = processor.GetMinAltitude();
    TAltitudeSectionOffset const headerSize = sizeof(version) + sizeof(minAltitude) + sizeof(TAltitudeSectionOffset);
    TAltitudeSectionOffset const featuresTableOffset = headerSize + sizeof(altitudeAvailabilitySize) + altitudeAvailabilitySize;
    TAltitudeSectionOffset const altitudeInfoOffset = featuresTableOffset + featuresTableSize;
    SerializeHeader(version, processor.GetMinAltitude(), altitudeInfoOffset, w);

    // Coping parts of altitude sections to mwm.
    MoveFileToAltitudeSection(altitudeAvailabilityPath, altitudeAvailabilitySize, w);
    MoveFileToAltitudeSection(featuresTablePath, featuresTableSize, w);
    MoveFileToAltitudeSection(altitudeInfoPath, altitudeInfoSize, w);
  }
  catch (RootException const & e)
  {
    LOG(LERROR, ("An exception happend while creating", ALTITUDE_FILE_TAG, "section. ", e.what()));
  }
}
}  // namespace routing
