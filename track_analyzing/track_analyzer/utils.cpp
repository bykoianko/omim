#include "track_analyzing/track_analyzer/utils.hpp"

#include "track_analyzing/log_parser.hpp"

#include "storage/country_info_getter.hpp"
#include "storage/routing_helpers.hpp"

#include "geometry/tree4d.hpp"

#include "platform/platform.hpp"

#include "base/logging.hpp"

#include <sstream>

namespace
{
void PrintMap(std::string const & name, std::map<std::string, uint32_t> const & mapping,
              std::ostringstream & ss)
{
  uint32_t allValues = 0;
  for (auto const & kv : mapping)
    allValues += kv.second;

  ss << name << '\n';
  if (allValues == 0)
  {
    ss << "map is emplty." << std::endl;
    return;
  }


  for (auto const & kv : mapping)
    ss << kv.first << ":" << kv.second << ", " << static_cast<double>(kv.second) / allValues << "%" << "\n";
  ss << "\n\n" << std::endl;
}
}

namespace track_analyzing
{
using namespace routing;
using namespace std;
using namespace storage;

void ParseTracks(string const & logFile, shared_ptr<NumMwmIds> const & numMwmIds,
                 MwmToTracks & mwmToTracks)
{
  Platform const & platform = GetPlatform();
  string const dataDir = platform.WritableDir();
  unique_ptr<CountryInfoGetter> countryInfoGetter =
      CountryInfoReader::CreateCountryInfoReader(platform);
  unique_ptr<m4::Tree<NumMwmId>> mwmTree = MakeNumMwmTree(*numMwmIds, *countryInfoGetter);

  LOG(LINFO, ("Parsing", logFile));
  LogParser parser(numMwmIds, move(mwmTree), dataDir);
  parser.Parse(logFile, mwmToTracks);
}

string DebugPrint(Stat const & s)
{
  std::ostringstream ss;
  ss << "Stat [ m_totalUserNum == " << s.m_totalUserNum
     << ", m_russianUserNum == " << s.m_russianUserNum
     << ", m_totalDataPointNum == " << s.m_totalDataPointNum
     << ", m_russianDataPointNum == " << s.m_russianDataPointNum << " ]" << "\n\n";

  PrintMap("Mwm to total users number:", s.m_mwmToTotalUser, ss);
  PrintMap("Mwm to total data points number:", s.m_mwmToTotalDataPointNum, ss);
  PrintMap("Country name to total users number:", s.m_countryToTotalUser, ss);
  PrintMap("Country name to data points number:", s.m_countryToTotalDataPointNum, ss);

  return ss.str();
}
}  // namespace track_analyzing
