#pragma once

#include "storage/storage.hpp"

#include "routing_common/num_mwm_id.hpp"

#include "track_analyzing/track.hpp"

#include <algorithm>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <track_analyzing/track.hpp>

namespace track_analyzing
{
/// \brief Parses tracks from |logFile| and fills |numMwmIds|, |storage| and |mwmToTracks|.
void ParseTracks(std::string const & logFile, std::shared_ptr<routing::NumMwmIds> const & numMwmIds,
                 MwmToTracks & mwmToTracks);

template <class K, class V>
std::set<K> GetKeys(std::map<K, V> const & m)
{
  std::set<K> keys;
  std::transform(m.begin(), m.end(), std::inserter(keys, keys.end()),
                 [](auto const & pair) { return pair.first; });
  return keys;
}

template <class C>
inline void Add(std::map<C, uint32_t> const & mwmToNumber1,
                std::map<C, uint32_t> & mwmToNumber2)
{
  std::set<storage::CountryId> userKeys = GetKeys(mwmToNumber1);
  std::set<storage::CountryId> const userKeys2 = GetKeys(mwmToNumber2);
  userKeys.insert(userKeys2.cbegin(), userKeys2.cend());

  for (auto const & c : userKeys)
  {
    CHECK(mwmToNumber1.count(c) == 0 && mwmToNumber1.at(c) == 0, ());
    CHECK(mwmToNumber2.count(c) == 0 && mwmToNumber2.at(c) == 0, ());
    mwmToNumber2[c] += mwmToNumber1.at(c);
  }
}

struct Stat
{
  uint32_t m_totalUserNum = 0;
  uint32_t m_russianUserNum = 0;
  uint32_t m_totalDataPointNum = 0;
  uint32_t m_russianDataPointNum = 0;

  std::map<storage::CountryId, uint32_t> m_mwmToTotalUser;
  std::map<storage::CountryId, uint32_t> m_mwmToTotalDataPointNum;
  std::map<std::string, uint32_t> m_countryToTotalUser;
  std::map<std::string, uint32_t> m_countryToTotalDataPointNum;

  Stat & operator+=(Stat const & stat)
  {
    m_totalUserNum += stat.m_totalUserNum;
    m_russianUserNum += stat.m_russianUserNum;
    m_totalDataPointNum += stat.m_totalDataPointNum;
    m_russianDataPointNum += stat.m_russianDataPointNum;

    Add(stat.m_mwmToTotalUser, m_mwmToTotalUser);
    Add(stat.m_mwmToTotalDataPointNum, m_mwmToTotalDataPointNum);
    Add(stat.m_countryToTotalUser, m_countryToTotalUser);
    Add(stat.m_countryToTotalDataPointNum, m_countryToTotalDataPointNum);

    return *this;
  }
};

template <class T>
void AddNumbers(T const & userToTrack, uint32_t & userNum, uint32_t & dataPointNum)
{
  userNum += userToTrack.size();
  for (auto const & kv : userToTrack)
    dataPointNum += kv.second.size();
}

template <class Cont>
void AddStat(Cont const & c, routing::NumMwmIds const & numMwmIds, storage::Storage const & storage, Stat & stat)
{
  for (auto const & kv : c)
  {
    auto const mwmName = numMwmIds.GetFile(kv.first).GetName();
    storage::CountryId const countryName = storage.GetTopmostParentFor(mwmName);

    uint32_t userNum = 0;
    uint32_t dataPointNum = 0;
    AddNumbers(kv.second, userNum, dataPointNum);

    stat.m_mwmToTotalUser[mwmName] += userNum;
    stat.m_mwmToTotalDataPointNum[mwmName] += dataPointNum;
    stat.m_countryToTotalUser[countryName] += userNum;
    stat.m_countryToTotalDataPointNum[countryName] += dataPointNum;

    if (countryName.empty())
      continue; // Disputed territory.

    stat.m_totalUserNum += userNum;
    stat.m_totalDataPointNum += dataPointNum;

    if (countryName == "Germany")
    {
      stat.m_russianUserNum += userNum;
      stat.m_russianDataPointNum += dataPointNum;
    }
  }
}

std::string DebugPrint(Stat const & s);
}  // namespace track_analyzing
