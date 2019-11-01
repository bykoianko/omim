#pragma once

#include "storage/storage.hpp"

#include "routing_common/num_mwm_id.hpp"

#include "track_analyzing/track.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <track_analyzing/track.hpp>

namespace track_analyzing
{
/// \brief Parses tracks from |logFile| and fills |numMwmIds|, |storage| and |mwmToTracks|.
void ParseTracks(std::string const & logFile, std::shared_ptr<routing::NumMwmIds> const & numMwmIds,
                 MwmToTracks & mwmToTracks);

struct Stat
{
  uint32_t m_totalUserNum = 0;
  uint32_t m_russianUserNum = 0;
  uint32_t m_totalDataPointNum = 0;
  uint32_t m_russianDataPointNum = 0;

  Stat & operator+=(Stat const & stat)
  {
    m_totalUserNum += stat.m_totalUserNum;
    m_russianUserNum += stat.m_russianUserNum;
    m_totalDataPointNum += stat.m_totalDataPointNum;
    m_russianDataPointNum += stat.m_russianDataPointNum;

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
    storage::CountryId const countryId = storage.GetTopmostParentFor(numMwmIds.GetFile(kv.first).GetName());
    if (countryId.empty())
      continue; // Disputed territory.

    AddNumbers(kv.second, stat.m_totalUserNum, stat.m_totalDataPointNum);
    if (countryId == "Russian Federation")
      AddNumbers(kv.second, stat.m_russianUserNum, stat.m_russianDataPointNum);
  }
}

std::string DebugPrint(Stat const & s);
}  // namespace track_analyzing
