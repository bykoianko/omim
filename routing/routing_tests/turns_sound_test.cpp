#include "testing/testing.hpp"

#include "routing/turns_sound.hpp"
#include "routing/turns_sound_settings.hpp"

#include "platform/location.hpp"

namespace
{
// A error to compare two double after conversion feet to meters.
double const kEps = 1.;
// A error to compare two doubles which are almost equal.
double const kSmallEps = .001;
}  // namespace

namespace routing
{
namespace turns
{
namespace sound
{
using namespace location;

UNIT_TEST(TurnNotificationSettingsMetersTest)
{
  Settings const settings(20 /* notificationTimeSeconds */,
                          200 /* minNotificationDistanceUnits */,
                          700 /* maxNotificationDistanceUnits */,
                          {100, 200, 300, 400, 500, 600, 700} /* soundedDistancesUnits */,
                          ::Settings::Metric /* lengthUnits */);

  TEST(settings.IsValid(), ());
  TEST(my::AlmostEqualAbs(
      settings.ConvertMetersPerSecondToUnitsPerSecond(20.), 20., kEps), ());
  TEST(my::AlmostEqualAbs(
      settings.ConvertMetersPerSecondToUnitsPerSecond(0.), 0., kEps), ());
  TEST(my::AlmostEqualAbs(settings.ConvertUnitsToMeters(300. /* distanceInUnits */), 300., kEps), ());
  TEST_EQUAL(settings.RoundByPresetSoundedDistancesUnits(300 /* distanceInUnits */), 300, ());
  TEST_EQUAL(settings.RoundByPresetSoundedDistancesUnits(0 /* distanceInUnits */), 100, ());

  TEST(my::AlmostEqualAbs(settings.ComputeTurnDistance(0. /* distanceInUnits */), 200., kSmallEps), ());
  TEST(my::AlmostEqualAbs(settings.ComputeTurnDistance(10. /* distanceInUnits */), 200., kSmallEps), ());
  TEST(my::AlmostEqualAbs(settings.ComputeTurnDistance(20. /* distanceInUnits */), 400., kSmallEps), ());
  TEST(my::AlmostEqualAbs(settings.ComputeTurnDistance(35. /* distanceInUnits */), 700., kSmallEps), ());
  TEST(my::AlmostEqualAbs(settings.ComputeTurnDistance(200. /* distanceInUnits */), 700., kSmallEps), ());
}

UNIT_TEST(TurnNotificationSettingsFeetTest)
{
  Settings const settings(20 /* notificationTimeSeconds */,
                          500 /* minNotificationDistanceUnits */,
                          2000 /* maxNotificationDistanceUnits */,
                          {200, 400, 600, 800, 1000, 1500, 2000} /* soundedDistancesUnits */,
                          ::Settings::Foot /* lengthUnits */);

  TEST(settings.IsValid(), ());
  TEST(my::AlmostEqualAbs(
      settings.ConvertMetersPerSecondToUnitsPerSecond(20.), 65., kEps), ());
  TEST(my::AlmostEqualAbs(
      settings.ConvertMetersPerSecondToUnitsPerSecond(0.), 0., kEps), ());
  TEST(my::AlmostEqualAbs(settings.ConvertUnitsToMeters(300. /* distanceInUnits */), 91., kEps), ());
  TEST_EQUAL(settings.RoundByPresetSoundedDistancesUnits(500 /* distanceInUnits */), 600, ());
  TEST_EQUAL(settings.RoundByPresetSoundedDistancesUnits(0 /* distanceInUnits */), 200, ());

  TEST(my::AlmostEqualAbs(settings.ComputeTurnDistance(0. /* distanceInUnits */), 500., kSmallEps), ());
  TEST(my::AlmostEqualAbs(settings.ComputeTurnDistance(10. /* distanceInUnits */), 500., kSmallEps), ());
  TEST(my::AlmostEqualAbs(settings.ComputeTurnDistance(30. /* distanceInUnits */), 600., kSmallEps), ());
  TEST(my::AlmostEqualAbs(settings.ComputeTurnDistance(40. /* distanceInUnits */), 800., kSmallEps), ());
  TEST(my::AlmostEqualAbs(settings.ComputeTurnDistance(200. /* distanceInUnits */), 2000., kSmallEps), ());
}

UNIT_TEST(TurnNotificationSettingsNotValidTest)
{
  Settings settings1(20 /* notificationTimeSeconds */, 500 /* minNotificationDistanceUnits */,
                     2000 /* maxNotificationDistanceUnits */,
                     {200, 400, 800, 600, 1000, 1500, 2000} /* soundedDistancesUnits */,
                     ::Settings::Foot /* lengthUnits */);
  TEST(!settings1.IsValid(), ());

  Settings settings2(20 /* notificationTimeSeconds */, 5000 /* minNotificationDistanceUnits */,
                     2000 /* maxNotificationDistanceUnits */,
                     {200, 400, 600, 800, 1000, 1500, 2000} /* soundedDistancesUnits */,
                     ::Settings::Metric /* lengthUnits */);
  TEST(!settings2.IsValid(), ());

  Settings settings3;
  TEST(!settings3.IsValid(), ());
}

UNIT_TEST(TurnsSoundMetersTest)
{
  TurnsSound turnSound;
  turnSound.Enable(true);
  turnSound.SetLengthUnits(::Settings::Metric);
  string const engShortJson =
      "\
      {\
      \"in_600_meters\":\"In 600 meters.\",\
      \"make_a_right_turn\":\"Make a right turn.\"\
      }";
  turnSound.m_getTtsText.ForTestingSetLocaleWithJson(engShortJson);
  turnSound.m_settings.ForTestingSetNotificationTimeSecond(20);

  turnSound.Reset();
  turnSound.SetSpeedMetersPerSecond(30.);

  vector<TurnItemDist> turns = {{{5 /* idx */, TurnDirection::TurnRight}, 1000.}};
  vector<string> turnNotifications;

  // Starting nearing the turnItem.
  // 1000 meters till the turn. No sound notifications is required.
  turnSound.GenerateTurnSound(turns, turnNotifications);
  TEST(turnNotifications.empty(), ());

  // 700 meters till the turn. No sound notifications is required.
  turns.front().m_distMeters = 800.;
  turnSound.GenerateTurnSound(turns, turnNotifications);
  TEST(turnNotifications.empty(), ());

  // 749 meters till the turn. It's time to pronounce the first voice notification.
  // Why? The current speed is 30 meters per seconds. According to correctSettingsMeters
  // we need to play the first voice notification 20 seconds before the turn.
  // Besides that we need 5 seconds (but 150 meters maximum) for playing the notification.
  // So we start playing the first notification when the distance till the turn is less
  // then 20 seconds * 30 meters per seconds + 150 meters = 750 meters.
  turns.front().m_distMeters = 749.;
  turnSound.GenerateTurnSound(turns, turnNotifications);
  vector<string> const expectedNotification1 = {{"In 600 meters. Make a right turn."}};
  TEST_EQUAL(turnNotifications, expectedNotification1, ());

  // 650 meters till the turn. No sound notifications is required.
  turns.front().m_distMeters = 650.;
  turnSound.GenerateTurnSound(turns, turnNotifications);
  TEST(turnNotifications.empty(), ());

  turnSound.SetSpeedMetersPerSecond(32.);

  // 200 meters till the turn. No sound notifications is required.
  turns.front().m_distMeters = 200.;
  turnSound.GenerateTurnSound(turns, turnNotifications);
  TEST(turnNotifications.empty(), ());

  // 150 meters till the turn. No sound notifications is required.
  turns.front().m_distMeters = 150.;
  turnSound.GenerateTurnSound(turns, turnNotifications);
  TEST(turnNotifications.empty(), ());

  // 149 meters till the turn. It's time to pronounce the second voice notification.
  turns.front().m_distMeters = 149.;
  turnSound.GenerateTurnSound(turns, turnNotifications);
  vector<string> const expectedNotification2 = {{"Make a right turn."}};
  TEST_EQUAL(turnNotifications, expectedNotification2, ());

  // 99 meters till the turn again. No sound notifications is required.
  turns.front().m_distMeters = 99.;
  turnSound.GenerateTurnSound(turns, turnNotifications);
  TEST(turnNotifications.empty(), ());

  // 50 meters till the turn. No sound notifications is required.
  turns.front().m_distMeters = 50.;
  turnSound.GenerateTurnSound(turns, turnNotifications);
  TEST(turnNotifications.empty(), ());

  // 0 meters till the turn. No sound notifications is required.
  turns.front().m_distMeters = 0.;
  turnSound.GenerateTurnSound(turns, turnNotifications);
  TEST(turnNotifications.empty(), ());

  TEST(turnSound.IsEnabled(), ());
}

// Test case:
// - Two turns;
// - They are close to each other;
// So the first notification of the second turn shall be skipped.
UNIT_TEST(TurnsSoundMetersTwoTurnsTest)
{
  TurnsSound turnSound;
  turnSound.Enable(true);
  turnSound.SetLengthUnits(::Settings::Metric);
  string const engShortJson =
      "\
      {\
      \"in_600_meters\":\"In 600 meters.\",\
      \"make_a_sharp_right_turn\":\"Make a sharp right turn.\",\
      \"enter_the_roundabout\":\"Enter the roundabout.\"\
      }";
  turnSound.m_getTtsText.ForTestingSetLocaleWithJson(engShortJson);
  turnSound.m_settings.ForTestingSetNotificationTimeSecond(20);

  turnSound.Reset();
  turnSound.SetSpeedMetersPerSecond(35.);

  vector<TurnItemDist> turns = {{{5 /* idx */, TurnDirection::TurnSharpRight}, 900.}};
  vector<string> turnNotifications;

  // Starting nearing the first turn.
  // 900 meters till the turn. No sound notifications is required.
  turnSound.GenerateTurnSound(turns, turnNotifications);
  TEST(turnNotifications.empty(), ());

  // 750 meters till the turn. It's time to pronounce the first voice notification.
  // The speed is high.
  // The compensation of kStartBeforeSeconds/kMinStartBeforeMeters/kMaxStartBeforeMeters is not enough.
  // The user will be closer to the turn while pronouncing despite the compensation.
  // So it should be pronounced "In 600 meters."
  turns.front().m_distMeters = 750.;
  turnSound.GenerateTurnSound(turns, turnNotifications);
  vector<string> const expectedNotification1 = {{"In 600 meters. Make a sharp right turn."}};
  TEST_EQUAL(turnNotifications, expectedNotification1, ());

  turnSound.SetSpeedMetersPerSecond(32.);

  // 150 meters till the turn. No sound notifications is required.
  turns.front().m_distMeters = 150.;
  turnSound.GenerateTurnSound(turns, turnNotifications);
  TEST(turnNotifications.empty(), ());

  // 99 meters till the turn. It's time to pronounce the second voice notification.
  turns.front().m_distMeters = 99.;
  turnSound.GenerateTurnSound(turns, turnNotifications);
  vector<string> const expectedNotification2 = {{"Make a sharp right turn."}};
  TEST_EQUAL(turnNotifications, expectedNotification2, ());

  turnSound.SetSpeedMetersPerSecond(10.);

  // 0 meters till the turn. No sound notifications is required.
  turns.front().m_distMeters = 0.;
  turnSound.GenerateTurnSound(turns, turnNotifications);
  TEST(turnNotifications.empty(), ());

  vector<TurnItemDist> turns2 = {{{11 /* idx */, TurnDirection::EnterRoundAbout,
                                         2 /* exitNum */}, 60.}};

  // Starting nearing the second turn.
  turnSound.GenerateTurnSound(turns2, turnNotifications);
  TEST(turnNotifications.empty(), ());

  // 40 meters till the second turn. It's time to pronounce the second voice notification
  // without the first one.
  turns2.front().m_distMeters = 40.;
  turnSound.GenerateTurnSound(turns2, turnNotifications);
  vector<string> const expectedNotification3 = {{"Enter the roundabout."}};
  TEST_EQUAL(turnNotifications, expectedNotification3, ());

  TEST(turnSound.IsEnabled(), ());
}

UNIT_TEST(TurnsSoundFeetTest)
{
  TurnsSound turnSound;
  turnSound.Enable(true);
  turnSound.SetLengthUnits(::Settings::Foot);
  string const engShortJson =
      "\
      {\
      \"in_2000_feet\":\"In 2000 feet.\",\
      \"enter_the_roundabout\":\"Enter the roundabout.\"\
      }";
  turnSound.m_getTtsText.ForTestingSetLocaleWithJson(engShortJson);
  turnSound.m_settings.ForTestingSetNotificationTimeSecond(20);

  turnSound.Reset();
  turnSound.SetSpeedMetersPerSecond(30.);

  vector<TurnItemDist> turns = {{{7 /* idx */, TurnDirection::EnterRoundAbout,
                                  3 /* exitNum */}, 1000.}};
  vector<string> turnNotifications;

  // Starting nearing the turnItem.
  // 1000 meters till the turn. No sound notifications is required.
  turnSound.GenerateTurnSound(turns, turnNotifications);
  TEST(turnNotifications.empty(), ());

  // 800 meters till the turn. No sound notifications is required.
  turns.front().m_distMeters = 800.;
  turnSound.GenerateTurnSound(turns, turnNotifications);
  TEST(turnNotifications.empty(), ());

  // 749 meters till the turn. It's time to pronounce the first voice notification.
  // Why? The current speed is 30 meters per seconds. According to correctSettingsMeters
  // we need to play the first voice notification 20 seconds before the turn.
  // Besides that we need 5 seconds (but 100 meters maximum) for playing the notification.
  // So we start playing the first notification when the distance till the turn is less
  // then 20 seconds * 30 meters per seconds + 150 meters = 750 meters.
  turns.front().m_distMeters = 749.;
  turnSound.GenerateTurnSound(turns, turnNotifications);
  vector<string> const expectedNotification1 = {{"In 2000 feet. Enter the roundabout."}};
  TEST_EQUAL(turnNotifications, expectedNotification1, ());

  // 650 meters till the turn. No sound notifications is required.
  turns.front().m_distMeters = 650.;
  turnSound.GenerateTurnSound(turns, turnNotifications);
  TEST(turnNotifications.empty(), ());

  // 200 meters till the turn. No sound notifications is required.
  turns.front().m_distMeters = 200.;
  turnSound.GenerateTurnSound(turns, turnNotifications);
  TEST(turnNotifications.empty(), ());

  // 150 meters till the turn. No sound notifications is required.
  turns.front().m_distMeters = 150.;
  turnSound.GenerateTurnSound(turns, turnNotifications);
  TEST(turnNotifications.empty(), ());

  // 149 meters till the turn. It's time to pronounce the second voice notification.
  turns.front().m_distMeters = 149.;
  turnSound.GenerateTurnSound(turns, turnNotifications);
  vector<string> const expectedNotification2 = {{"Enter the roundabout."}};
  TEST_EQUAL(turnNotifications, expectedNotification2, ());

  // 149 meters till the turn again. No sound notifications is required.
  turns.front().m_distMeters = 149.;
  turnSound.GenerateTurnSound(turns, turnNotifications);
  TEST(turnNotifications.empty(), ());

  // 50 meters till the turn. No sound notifications is required.
  turns.front().m_distMeters = 50.;
  turnSound.GenerateTurnSound(turns, turnNotifications);
  TEST(turnNotifications.empty(), ());

  // 0 meters till the turn. No sound notifications is required.
  turns.front().m_distMeters = 0.;
  turnSound.GenerateTurnSound(turns, turnNotifications);
  TEST(turnNotifications.empty(), ());

  TEST(turnSound.IsEnabled(), ());
}

UNIT_TEST(TurnsSoundComposedTurnTest)
{
  TurnsSound turnSound;
  turnSound.Enable(true);
  turnSound.SetLengthUnits(::Settings::Metric);
  string const engShortJson =
      "\
      {\
      \"in_600_meters\":\"In 600 meters.\",\
      \"make_a_right_turn\":\"Turn right.\",\
      \"enter_the_roundabout\":\"Enter the roundabout.\",\
      \"then\":\"Then.\",\
      \"you_have_reached_the_destination\":\"You have reached the destination.\"\
      }";
  turnSound.m_getTtsText.ForTestingSetLocaleWithJson(engShortJson);
  turnSound.m_settings.ForTestingSetNotificationTimeSecond(30);

  turnSound.Reset();
  turnSound.SetSpeedMetersPerSecond(20.);
  vector<string> turnNotifications;

  // Starting nearing the first turn.
  // 800 meters till the first turn.
  vector<TurnItemDist> const turns1 = {{{5 /* idx */, TurnDirection::TurnRight}, 800. /* m_distMeters */},
                                       {{10 /* idx */, TurnDirection::EnterRoundAbout}, 1000. /* m_distMeters */}};
  turnSound.GenerateTurnSound(turns1, turnNotifications);
  TEST(turnNotifications.empty(), ());

  // 620 meters till the first turn.
  turnNotifications.clear();
  vector<TurnItemDist> const turns2 = {{{5 /* idx */, TurnDirection::TurnRight}, 620. /* m_distMeters */},
                                       {{10 /* idx */, TurnDirection::EnterRoundAbout}, 820. /* m_distMeters */}};
  vector<string> const expectedNotification2 = {{"In 600 meters. Turn right."},
                                                {"Then. Enter the roundabout."}};
  turnSound.GenerateTurnSound(turns2, turnNotifications);
  TEST_EQUAL(turnNotifications, expectedNotification2, ());

  // 300 meters till the first turn.
  turnNotifications.clear();
  vector<TurnItemDist> const turns3 = {{{5 /* idx */, TurnDirection::TurnRight}, 300. /* m_distMeters */},
                                       {{10 /* idx */, TurnDirection::EnterRoundAbout}, 500. /* m_distMeters */}};
  turnSound.GenerateTurnSound(turns3, turnNotifications);
  TEST(turnNotifications.empty(), ());

  // 20 meters till the first turn.
  turnNotifications.clear();
  vector<TurnItemDist> const turns4 = {{{5 /* idx */, TurnDirection::TurnRight}, 20. /* m_distMeters */},
                                       {{10 /* idx */, TurnDirection::EnterRoundAbout}, 220. /* m_distMeters */}};
  vector<string> const expectedNotification4 = {{"Turn right."},
                                                {"Then. Enter the roundabout."}};
  turnSound.GenerateTurnSound(turns4, turnNotifications);
  TEST_EQUAL(turnNotifications, expectedNotification4, ());

  // After the first turn.
  turnNotifications.clear();
  vector<TurnItemDist> const turns5 = {{{10 /* idx */, TurnDirection::EnterRoundAbout}, 180. /* m_distMeters */},
                                       {{15 /* idx */, TurnDirection::ReachedYourDestination}, 1180. /* m_distMeters */}};
  turnSound.GenerateTurnSound(turns5, turnNotifications);
  TEST(turnNotifications.empty(), ());

  // Just before the second turn.
  turnNotifications.clear();
  vector<TurnItemDist> const turns6 = {{{10 /* idx */, TurnDirection::EnterRoundAbout}, 10. /* m_distMeters */},
                                       {{15 /* idx */, TurnDirection::ReachedYourDestination}, 1010. /* m_distMeters */}};
  vector<string> const expectedNotification6 = {{"Enter the roundabout."}};
  turnSound.GenerateTurnSound(turns6, turnNotifications);
  TEST_EQUAL(turnNotifications, expectedNotification6, ());
}

UNIT_TEST(TurnsSoundRoundaboutTurnTest)
{
  TurnsSound turnSound;
  turnSound.Enable(true);
  turnSound.SetLengthUnits(::Settings::Metric);
  string const engShortJson =
      "\
      {\
      \"enter_the_roundabout\":\"Enter the roundabout.\",\
      \"leave_the_roundabout\":\"Leave the roundabout.\",\
      \"take_the_1st_exit\":\"Take the first exit.\",\
      \"take_the_2nd_exit\":\"Take the second exit.\",\
      \"take_the_4th_exit\":\"Take the fourth exit.\",\
      \"in_600_meters\":\"In 600 meters.\",\
      \"then\":\"Then.\"\
      }";
  turnSound.m_getTtsText.ForTestingSetLocaleWithJson(engShortJson);
  turnSound.m_settings.ForTestingSetNotificationTimeSecond(30);

  turnSound.Reset();
  turnSound.SetSpeedMetersPerSecond(20.);
  vector<string> turnNotifications;

  // Starting nearing the first turn.
  // 1000 meters till the first turn.
  vector<TurnItemDist> const turns1 = {{{5 /* idx */, TurnDirection::EnterRoundAbout, 2 /* m_exitNum */},
                                        1000. /* m_distMeters */},
                                       {{10 /* idx */, TurnDirection::LeaveRoundAbout, 2 /* m_exitNum */},
                                        2000. /* m_distMeters */}};
  turnSound.GenerateTurnSound(turns1, turnNotifications);
  TEST(turnNotifications.empty(), ());

  // 620 meters till the first turn.
  vector<TurnItemDist> const turns2 = {{{5 /* idx */, TurnDirection::EnterRoundAbout, 2 /* m_exitNum */},
                                        620. /* m_distMeters */},
                                       {{10 /* idx */, TurnDirection::LeaveRoundAbout, 2 /* m_exitNum */},
                                        1620. /* m_distMeters */}};
  vector<string> const expectedNotification2 = {{"In 600 meters. Enter the roundabout."},
                                                {"Then. Take the second exit."}};
  turnSound.GenerateTurnSound(turns2, turnNotifications);
  TEST_EQUAL(turnNotifications, expectedNotification2, ());

  // 3 meters till the first turn.
  vector<TurnItemDist> const turns3 = {{{5 /* idx */, TurnDirection::EnterRoundAbout, 2 /* m_exitNum */},
                                        3. /* m_distMeters */},
                                       {{10 /* idx */, TurnDirection::LeaveRoundAbout, 2 /* m_exitNum */},
                                        1003. /* m_distMeters */}};
  vector<string> const expectedNotification3 = {{"Enter the roundabout."},
                                                {"Then. Take the second exit."}};
  turnSound.GenerateTurnSound(turns3, turnNotifications);
  TEST_EQUAL(turnNotifications, expectedNotification3, ());

  // 900 meters till the second turn.
  vector<TurnItemDist> const turns4 = {{{10 /* idx */, TurnDirection::LeaveRoundAbout, 2 /* m_exitNum */},
                                        900. /* m_distMeters */},
                                       {{15 /* idx */, TurnDirection::EnterRoundAbout, 1 /* m_exitNum */},
                                        1900. /* m_distMeters */}};
  turnSound.GenerateTurnSound(turns4, turnNotifications);
  TEST(turnNotifications.empty(), ());

  // 300 meters till the second turn.
  vector<TurnItemDist> const turns5 = {{{10 /* idx */, TurnDirection::LeaveRoundAbout, 2 /* m_exitNum */},
                                        300. /* m_distMeters */},
                                       {{15 /* idx */, TurnDirection::EnterRoundAbout, 1 /* m_exitNum */},
                                        1300. /* m_distMeters */}};
  turnSound.GenerateTurnSound(turns5, turnNotifications);
  TEST(turnNotifications.empty(), ());

  // 3 meters till the second turn.
  vector<TurnItemDist> const turns6 = {{{10 /* idx */, TurnDirection::LeaveRoundAbout, 2 /* m_exitNum */},
                                        3. /* m_distMeters */},
                                       {{15 /* idx */, TurnDirection::EnterRoundAbout, 1 /* m_exitNum */},
                                        1003. /* m_distMeters */}};
  vector<string> const expectedNotification6 = {{"Leave the roundabout."}};
  turnSound.GenerateTurnSound(turns6, turnNotifications);
  TEST_EQUAL(turnNotifications, expectedNotification6, ());

  // 5 meters till the third turn.
  vector<TurnItemDist> const turns7 = {{{15 /* idx */, TurnDirection::EnterRoundAbout, 1 /* m_exitNum */},
                                        5. /* m_distMeters */},
                                       {{20 /* idx */, TurnDirection::LeaveRoundAbout, 1 /* m_exitNum */},
                                        1005. /* m_distMeters */}};
  vector<string> const expectedNotification7 = {{"Enter the roundabout."},
                                                {"Then. Take the first exit."}};
  turnSound.GenerateTurnSound(turns7, turnNotifications); // The first notification fast forwarding.
  turnSound.GenerateTurnSound(turns7, turnNotifications);
  TEST_EQUAL(turnNotifications, expectedNotification7, ());

  // 900 meters till the 4th turn.
  turnSound.Reset();
  vector<TurnItemDist> const turns8 = {{{25 /* idx */, TurnDirection::EnterRoundAbout, 4 /* m_exitNum */},
                                        900. /* m_distMeters */},
                                       {{30 /* idx */, TurnDirection::LeaveRoundAbout, 4 /* m_exitNum */},
                                        1200. /* m_distMeters */}};
  turnSound.GenerateTurnSound(turns8, turnNotifications);
  TEST(turnNotifications.empty(), ());

  // 620 meters till the 4th turn.
  vector<TurnItemDist> const turns9 = {{{25 /* idx */, TurnDirection::EnterRoundAbout, 4 /* m_exitNum */},
                                        620. /* m_distMeters */},
                                       {{30 /* idx */, TurnDirection::LeaveRoundAbout, 4 /* m_exitNum */},
                                        920. /* m_distMeters */}};
  vector<string> const expectedNotification9 = {{"In 600 meters. Enter the roundabout."},
                                                {"Then. Take the fourth exit."}};
  turnSound.GenerateTurnSound(turns9, turnNotifications);
  TEST_EQUAL(turnNotifications, expectedNotification9, ());
}
}  // namespace sound
}  // namespace turns
}  // namespace routing
