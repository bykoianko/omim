#include "track_analyzing/serialization.hpp"
#include "track_analyzing/track.hpp"
#include "track_analyzing/track_analyzer/utils.hpp"
#include "track_analyzing/track_matcher.hpp"
#include "track_analyzing/utils.hpp"

#include "routing_common/num_mwm_id.hpp"

#include "storage/routing_helpers.hpp"
#include "storage/storage.hpp"

#include "coding/file_reader.hpp"
#include "coding/file_writer.hpp"
#include "coding/zlib.hpp"

#include "platform/platform.hpp"

#include "base/assert.hpp"
#include "base/file_name_utils.hpp"
#include "base/logging.hpp"
#include "base/timer.hpp"

#include <algorithm>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

using namespace routing;
using namespace std;
using namespace storage;
using namespace track_analyzing;

namespace
{
using Iter = typename vector<string>::iterator;

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

string DebugPrint(Stat const & s)
{
  ostringstream ss;
  ss << "Stat [ m_totalUserNum == " << s.m_totalUserNum
      << ", m_russianUserNum == " << s.m_russianUserNum
      << ", m_totalDataPointNum == " << s.m_totalDataPointNum
      << ", m_russianDataPointNum == " << s.m_russianDataPointNum  << " ]" << endl;
  return ss.str();
}

void MatchTracks(MwmToTracks const & mwmToTracks, storage::Storage const & storage,
                 NumMwmIds const & numMwmIds, MwmToMatchedTracks & mwmToMatchedTracks)
{
  base::Timer timer;

  uint64_t tracksCount = 0;
  uint64_t pointsCount = 0;
  uint64_t nonMatchedPointsCount = 0;

  auto processMwm = [&](string const & mwmName, UserToTrack const & userToTrack) {
    auto const countryFile = platform::CountryFile(mwmName);
    auto const mwmId = numMwmIds.GetId(countryFile);
    TrackMatcher matcher(storage, mwmId, countryFile);

    auto & userToMatchedTracks = mwmToMatchedTracks[mwmId];

    for (auto const & it : userToTrack)
    {
      string const & user = it.first;
      auto & matchedTracks = userToMatchedTracks[user];
      try
      {
        matcher.MatchTrack(it.second, matchedTracks);
      }
      catch (RootException const & e)
      {
        LOG(LERROR, ("Can't match track for mwm:", mwmName, ", user:", user));
        LOG(LERROR, ("  ", e.what()));
      }

      if (matchedTracks.empty())
        userToMatchedTracks.erase(user);
    }

    if (userToMatchedTracks.empty())
      mwmToMatchedTracks.erase(mwmId);

    tracksCount += matcher.GetTracksCount();
    pointsCount += matcher.GetPointsCount();
    nonMatchedPointsCount += matcher.GetNonMatchedPointsCount();

    LOG(LINFO, (numMwmIds.GetFile(mwmId).GetName(), ", users:", userToTrack.size(), ", tracks:",
                matcher.GetTracksCount(), ", points:", matcher.GetPointsCount(),
                ", non matched points:", matcher.GetNonMatchedPointsCount()));
  };

  ForTracksSortedByMwmName(mwmToTracks, numMwmIds, processMwm);

  LOG(LINFO,
      ("Matching finished, elapsed:", timer.ElapsedSeconds(), "seconds, tracks:", tracksCount,
       ", points:", pointsCount, ", non matched points:", nonMatchedPointsCount));
}

void AddNumbers(UserToTrack const & userToTrack, uint32_t & userNum, uint32_t & dataPointNum)
{
  userNum += userToTrack.size();
  for (auto const & kv : userToTrack)
    dataPointNum += kv.second.size();
}
}  // namespace

namespace track_analyzing
{
void CmdMatch(string const & logFile, string const & trackFile, shared_ptr<NumMwmIds> const & numMwmIds, Storage const & storage, Stat & stat)
{
  MwmToTracks mwmToTracks;
  ParseTracks(logFile, numMwmIds, mwmToTracks);

  for (auto const & kv : mwmToTracks)
  {
    CountryId const countryId = storage.GetTopmostParentFor(numMwmIds->GetFile(kv.first).GetName());
    if (countryId.empty())
      continue; // Disputed territory.

    AddNumbers(kv.second, stat.m_totalUserNum, stat.m_totalDataPointNum);
    if (countryId == "Russian Federation")
      AddNumbers(kv.second, stat.m_russianUserNum, stat.m_russianDataPointNum);
  }

  MwmToMatchedTracks mwmToMatchedTracks;
  MatchTracks(mwmToTracks, storage, *numMwmIds, mwmToMatchedTracks);

  FileWriter writer(trackFile, FileWriter::OP_WRITE_TRUNCATE);
  MwmToMatchedTracksSerializer serializer(numMwmIds);
  serializer.Serialize(mwmToMatchedTracks, writer);
  LOG(LINFO, ("Matched tracks were saved to", trackFile));
}

void CmdMatch(string const & logFile, string const & trackFile)
{
  LOG(LINFO, ("Matching", logFile));
  Storage storage;
  storage.RegisterAllLocalMaps(false /* enableDiffs */);
  shared_ptr<NumMwmIds> numMwmIds = CreateNumMwmIds(storage);
  Stat stat;
  CmdMatch(logFile, trackFile, numMwmIds, storage, stat);
  LOG(LINFO, ("CmdMatch stat.", stat));
}

void UnzipAndMatch(Iter begin, Iter end, string const & trackExt, Stat & stat)
{
  Storage storage;
  storage.RegisterAllLocalMaps(false /* enableDiffs */);
  shared_ptr<NumMwmIds> numMwmIds = CreateNumMwmIds(storage);
  for (auto it = begin; it != end; ++it)
  {
    auto & file = *it;
    string data;
    try
    {
      auto const r = GetPlatform().GetReader(file);
      r->ReadAsString(data);
    }
    catch (FileReader::ReadException const & e)
    {
      LOG(LWARNING, (e.what()));
      continue;
    }

    using Inflate = coding::ZLib::Inflate;
    Inflate inflate(Inflate::Format::GZip);
    string track;
    inflate(data.data(), data.size(), back_inserter(track));
    base::GetNameWithoutExt(file);
    try
    {
      FileWriter w(file);
      w.Write(track.data(), track.size());
    }
    catch (FileWriter::WriteException const & e)
    {
      LOG(LWARNING, (e.what()));
      continue;
    }

    CmdMatch(file, file + trackExt, numMwmIds, storage, stat);
    FileWriter::DeleteFileX(file);
  }
}

void CmdMatchDir(string const & logDir, string const & trackExt)
{
  Platform::EFileType fileType = Platform::FILE_TYPE_UNKNOWN;
  Platform::EError const result = Platform::GetFileType(logDir, fileType);

  if (result == Platform::ERR_FILE_DOES_NOT_EXIST)
  {
    LOG(LINFO, ("Directory doesn't exist", logDir));
    return;
  }

  if (result != Platform::ERR_OK)
  {
    LOG(LINFO, ("Can't get file type for", logDir));
    return;
  }

  if (fileType != Platform::FILE_TYPE_DIRECTORY)
  {
    LOG(LINFO, (logDir, "is not a directory."));
    return;
  }

  Platform::FilesList filesList;
  Platform::GetFilesRecursively(logDir, filesList);
  if (filesList.empty())
  {
    LOG(LINFO, (logDir, "is empty."));
    return;
  }

  auto const size = filesList.size();
  auto const hardwareConcurrency = static_cast<size_t>(thread::hardware_concurrency());
  CHECK_GREATER(hardwareConcurrency, 0, ("No available threads."));
  LOG(LINFO, ("Number of available threads =", hardwareConcurrency));
  auto const threadsCount = min(size, hardwareConcurrency);
  auto const blockSize = size / threadsCount;
  vector<thread> threads(threadsCount - 1);
  vector<Stat> stats(threadsCount);
  auto begin = filesList.begin();
  for (size_t i = 0; i < threadsCount - 1; ++i)
  {
    auto end = begin + blockSize;
    threads[i] = thread(UnzipAndMatch, begin, end, trackExt, ref(stats[i]));
    begin = end;
  }

  UnzipAndMatch(begin, filesList.end(), trackExt, ref(stats[threadsCount - 1]));
  for (auto & t : threads)
    t.join();

  Stat statSum;
  for (auto const & s : stats)
    statSum += s;
  LOG(LINFO, ("CmdMatchDir stat.", statSum));
}
}  // namespace track_analyzing
