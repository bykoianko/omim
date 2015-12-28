#include "testing/testing.hpp"

#include "test_mwm_set.hpp"

#include "indexer/mwm_set.hpp"

#include "base/macros.hpp"

#include "std/initializer_list.hpp"
#include "std/unordered_map.hpp"

using platform::CountryFile;
using platform::LocalCountryFile;
using tests::TestMwmSet;

using TMwmsInfo = unordered_map<string, shared_ptr<MwmInfo>>;

namespace
{
void GetMwmsInfo(MwmSet const & mwmSet, TMwmsInfo & mwmsInfo)
{
  vector<shared_ptr<MwmInfo>> mwmsInfoList;
  mwmSet.GetMwmsInfo(mwmsInfoList);

  mwmsInfo.clear();
  for (shared_ptr<MwmInfo> const & info : mwmsInfoList)
    mwmsInfo[info->GetCountryName()] = info;
}

void TestFilesPresence(TMwmsInfo const & mwmsInfo, initializer_list<string> const & expectedNames)
{
  TEST_EQUAL(expectedNames.size(), mwmsInfo.size(), ());
  for (string const & countryFileName : expectedNames)
    TEST_EQUAL(1, mwmsInfo.count(countryFileName), (countryFileName));
}
}  // namespace

UNIT_TEST(MwmSetSmokeTest)
{
  TestMwmSet mwmSet;
  TMwmsInfo mwmsInfo;

  UNUSED_VALUE(mwmSet.Register(LocalCountryFile::MakeForTesting("0")));
  UNUSED_VALUE(mwmSet.Register(LocalCountryFile::MakeForTesting("1")));
  UNUSED_VALUE(mwmSet.Register(LocalCountryFile::MakeForTesting("2")));
  mwmSet.Deregister(CountryFile("1"));

  GetMwmsInfo(mwmSet, mwmsInfo);
  TestFilesPresence(mwmsInfo, {"0", "2"});

  TEST(mwmsInfo["0"]->IsUpToDate(), ());
  TEST_EQUAL(mwmsInfo["0"]->m_maxScale, 0, ());
  TEST(mwmsInfo["2"]->IsUpToDate(), ());
  {
    MwmSet::MwmHandle const handle0 = mwmSet.GetMwmHandleByCountryFile(CountryFile("0"));
    MwmSet::MwmHandle const handle1 = mwmSet.GetMwmHandleByCountryFile(CountryFile("1"));
    TEST(handle0.IsAlive(), ());
    TEST(!handle1.IsAlive(), ());
  }

  UNUSED_VALUE(mwmSet.Register(LocalCountryFile::MakeForTesting("3")));

  GetMwmsInfo(mwmSet, mwmsInfo);
  TestFilesPresence(mwmsInfo, {"0", "2", "3"});

  TEST(mwmsInfo["0"]->IsUpToDate(), ());
  TEST_EQUAL(mwmsInfo["0"]->m_maxScale, 0, ());
  TEST(mwmsInfo["2"]->IsUpToDate(), ());
  TEST_EQUAL(mwmsInfo["2"]->m_maxScale, 2, ());
  TEST(mwmsInfo["3"]->IsUpToDate(), ());
  TEST_EQUAL(mwmsInfo["3"]->m_maxScale, 3, ());

  {
    MwmSet::MwmHandle const handle1 = mwmSet.GetMwmHandleByCountryFile(CountryFile("1"));
    TEST(!handle1.IsAlive(), ());
    mwmSet.Deregister(CountryFile("3"));
    UNUSED_VALUE(mwmSet.Register(LocalCountryFile::MakeForTesting("4")));
  }

  GetMwmsInfo(mwmSet, mwmsInfo);
  TestFilesPresence(mwmsInfo, {"0", "2", "4"});

  TEST(mwmsInfo["0"]->IsUpToDate(), ());
  TEST_EQUAL(mwmsInfo["0"]->m_maxScale, 0, ());
  TEST(mwmsInfo["2"]->IsUpToDate(), ());
  TEST_EQUAL(mwmsInfo["2"]->m_maxScale, 2, ());
  TEST(mwmsInfo["4"]->IsUpToDate(), ());
  TEST_EQUAL(mwmsInfo["4"]->m_maxScale, 4, ());

  UNUSED_VALUE(mwmSet.Register(LocalCountryFile::MakeForTesting("5")));

  GetMwmsInfo(mwmSet, mwmsInfo);
  TestFilesPresence(mwmsInfo, {"0", "2", "4", "5"});

  TEST_EQUAL(mwmsInfo.size(), 4, ());
  TEST(mwmsInfo["0"]->IsUpToDate(), ());
  TEST_EQUAL(mwmsInfo["0"]->m_maxScale, 0, ());
  TEST(mwmsInfo["2"]->IsUpToDate(), ());
  TEST_EQUAL(mwmsInfo["2"]->m_maxScale, 2, ());
  TEST(mwmsInfo["4"]->IsUpToDate(), ());
  TEST_EQUAL(mwmsInfo["4"]->m_maxScale, 4, ());
  TEST(mwmsInfo["5"]->IsUpToDate(), ());
  TEST_EQUAL(mwmsInfo["5"]->m_maxScale, 5, ());
}

UNIT_TEST(MwmSetIdTest)
{
  TestMwmSet mwmSet;
  TEST_EQUAL(MwmSet::RegResult::Success,
             mwmSet.Register(LocalCountryFile::MakeForTesting("3", 0 /* version */)).second, ());

  MwmSet::MwmId const id0 = mwmSet.GetMwmHandleByCountryFile(CountryFile("3")).GetId();
  MwmSet::MwmId const id1 = mwmSet.GetMwmHandleByCountryFile(CountryFile("3")).GetId();

  TEST(id0.IsAlive(), ());
  TEST(id1.IsAlive(), ());
  TEST_EQUAL(id0.GetInfo().get(), id1.GetInfo().get(), ());
  TEST_EQUAL(MwmInfo::STATUS_REGISTERED, id0.GetInfo()->GetStatus(), ());

  TEST(mwmSet.Deregister(CountryFile("3")), ());

  // Test that both id's are sour now.
  TEST(!id0.IsAlive(), ());
  TEST(!id1.IsAlive(), ());
  TEST_EQUAL(id0.GetInfo().get(), id1.GetInfo().get(), ());
  TEST_EQUAL(MwmInfo::STATUS_DEREGISTERED, id0.GetInfo()->GetStatus(), ());
}

UNIT_TEST(MwmSetLockAndIdTest)
{
  TestMwmSet mwmSet;
  MwmSet::MwmId id;

  {
    auto p = mwmSet.Register(LocalCountryFile::MakeForTesting("4"));
    MwmSet::MwmHandle handle = mwmSet.GetMwmHandleById(p.first);
    TEST(handle.IsAlive(), ());
    TEST_EQUAL(MwmSet::RegResult::Success, p.second, ("Can't register test mwm 4"));
    TEST_EQUAL(MwmInfo::STATUS_REGISTERED, handle.GetInfo()->GetStatus(), ());

    TEST(!mwmSet.Deregister(CountryFile("4")), ());  // It's not possible to remove mwm 4 right now.
    TEST(handle.IsAlive(), ());
    TEST_EQUAL(MwmInfo::STATUS_MARKED_TO_DEREGISTER, handle.GetInfo()->GetStatus(), ());
    id = handle.GetId();
    TEST(id.IsAlive(), ());
  }

  TEST(!id.IsAlive(), ());       // Mwm is not alive, so id is sour now.
  TEST(id.GetInfo().get(), ());  // But it's still possible to get an MwmInfo.
  TEST_EQUAL(MwmInfo::STATUS_DEREGISTERED, id.GetInfo()->GetStatus(), ());
  TEST_EQUAL(4, id.GetInfo()->m_maxScale, ());

  // It is not possible to lock mwm 4 because it is already deleted,
  // and it is not possible to get to it's info from mwmSet.
  MwmSet::MwmHandle handle = mwmSet.GetMwmHandleByCountryFile(CountryFile("4"));
  TEST(!handle.IsAlive(), ());
  TEST(!handle.GetId().IsAlive(), ());
  TEST(!handle.GetId().GetInfo().get(), ());
}
