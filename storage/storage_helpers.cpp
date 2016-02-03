#include "storage/storage_helpers.hpp"

#include "storage/country_info_getter.hpp"
#include "storage/storage.hpp"

namespace storage
{

bool IsPointCoveredByDownloadedMaps(m2::PointD const & position,
                                    Storage const & storage,
                                    CountryInfoGetter const & countryInfoGetter)
{
  return storage.IsNodeDownloaded(countryInfoGetter.GetRegionCountryId(position));
}

} // namespace storage
