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
struct KeyValue
{
  std::string m_key;
  uint32_t m_value = 0;
};

void PrintMap(std::string const & name, std::string const & descr,
              std::map<std::string, uint32_t> const & mapping, std::ostringstream & ss)
{
  std::vector<KeyValue> keyValues;
  keyValues.reserve(mapping.size());
  for (auto const & kv : mapping)
    keyValues.push_back({kv.first, kv.second});

  std::sort(keyValues.begin(), keyValues.end(),
            [](KeyValue const & a, KeyValue const & b) { return a.m_value > b.m_value; });

  uint32_t allValues = 0;
  for (auto const & kv : keyValues)
    allValues += kv.m_value;

  ss << descr << '\n';
  if (allValues == 0)
  {
    ss << "map is empty." << std::endl;
    return;
  }

  ss << name << ",number,percent";
  for (auto const & kv : keyValues)
  {
    if (kv.m_value == 0)
      continue;

    ss << kv.m_key << "," << kv.m_value << ","
       << 100.0 * static_cast<double>(kv.m_value) / allValues << "\n";
  }
  ss << "\n" << std::endl;
}
}  // namespace

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

  PrintMap("mwm", "Mwm to total users number:", s.m_mwmToTotalUsers, ss);
  PrintMap("mwm", "Mwm to total data points number:", s.m_mwmToTotalDataPoints, ss);
  PrintMap("country", "Country name to total users number:", s.m_countryToTotalUsers, ss);
  PrintMap("country", "Country name to data points number:", s.m_countryToTotalDataPoints, ss);

  return ss.str();
}
}  // namespace track_analyzing
