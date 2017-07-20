#include "testing/testing.hpp"

#include "routing/route.hpp"
#include "routing/routing_helpers.hpp"
#include "routing/turns.hpp"

#include "platform/location.hpp"

#include "geometry/mercator.hpp"
#include "geometry/point2d.hpp"

#include "std/string.hpp"
#include "std/vector.hpp"

using namespace routing;

namespace
{
static vector<m2::PointD> const kTestGeometry({{0, 0}, {0,1}, {1,1}, {1,2}, {1,3}});
static vector<Segment> const kTestSegments(
    {{0, 0, 0, true}, {0, 0, 1, true}, {0, 0, 2, true}, {0, 0, 3, true}});
static Route::TTurns const kTestTurns(
    {turns::TurnItem(1, turns::TurnDirection::TurnLeft),
     turns::TurnItem(2, turns::TurnDirection::TurnRight),
     turns::TurnItem(4, turns::TurnDirection::ReachedYourDestination)});
static Route::TStreets const kTestNames({{0, "Street1"}, {1, "Street2"}, {4, "Street3"}});
static Route::TTimes const kTestTimes({Route::TTimeItem(1, 5), Route::TTimeItem(3, 10),
                                      Route::TTimeItem(4, 15)});

location::GpsInfo GetGps(double x, double y)
{
  location::GpsInfo info;
  info.m_latitude = MercatorBounds::YToLat(y);
  info.m_longitude = MercatorBounds::XToLon(x);
  info.m_horizontalAccuracy = 2;
  info.m_speed = -1;
  return info;
}

void TestSegmentInfo(RouteSegment const & segmentInfo, turns::TurnDirection turn,
                     double distFromBeginningMerc, traffic::SpeedGroup speedGroup,
                     double timeFromBeginningS)
{
  TEST_EQUAL(segmentInfo.GetTurn().m_turn, turn, ());
  TEST_EQUAL(segmentInfo.GetDistFromBeginningMerc(), distFromBeginningMerc, ());
  TEST_EQUAL(segmentInfo.GetTraffic(), speedGroup, ());
  TEST_EQUAL(segmentInfo.GetTimeFromBeginningS(), timeFromBeginningS, ());
}
}  // namespace

UNIT_TEST(AddAdsentCountryToRouteTest)
{
  Route route("TestRouter");
  route.AddAbsentCountry("A");
  route.AddAbsentCountry("A");
  route.AddAbsentCountry("B");
  route.AddAbsentCountry("C");
  route.AddAbsentCountry("B");
  set<string> const & absent = route.GetAbsentCountries();
  TEST(absent.find("A") != absent.end(), ());
  TEST(absent.find("B") != absent.end(), ());
  TEST(absent.find("C") != absent.end(), ());
}

UNIT_TEST(DistanceToCurrentTurnTest)
{
  Route route("TestRouter");
  route.SetGeometry(kTestGeometry.begin(), kTestGeometry.end());
  vector<turns::TurnItem> turns(kTestTurns);
  route.SetTurnInstructions(move(turns));

  double distance;
  turns::TurnItem turn;

  route.GetCurrentTurn(distance, turn);
  TEST(my::AlmostEqualAbs(distance,
                          MercatorBounds::DistanceOnEarth(kTestGeometry[0], kTestGeometry[1]), 0.1),
                          ());
  TEST_EQUAL(turn, kTestTurns[0], ());

  route.MoveIterator(GetGps(0, 0.5));
  route.GetCurrentTurn(distance, turn);
  TEST(my::AlmostEqualAbs(distance,
                          MercatorBounds::DistanceOnEarth({0, 0.5}, kTestGeometry[1]), 0.1), ());
  TEST_EQUAL(turn, kTestTurns[0], ());

  route.MoveIterator(GetGps(1, 1.5));
  route.GetCurrentTurn(distance, turn);
  TEST(my::AlmostEqualAbs(distance,
                          MercatorBounds::DistanceOnEarth({1, 1.5}, kTestGeometry[4]), 0.1), ());
  TEST_EQUAL(turn, kTestTurns[2], ());

  route.MoveIterator(GetGps(1, 2.5));
  route.GetCurrentTurn(distance, turn);
  TEST(my::AlmostEqualAbs(distance,
                          MercatorBounds::DistanceOnEarth({1, 2.5}, kTestGeometry[4]), 0.1), ());
  TEST_EQUAL(turn, kTestTurns[2], ());
}

UNIT_TEST(NextTurnTest)
{
  Route route("TestRouter");
  route.SetGeometry(kTestGeometry.begin(), kTestGeometry.end());
  vector<turns::TurnItem> turns(kTestTurns);
  route.SetTurnInstructions(move(turns));

  double distance, nextDistance;
  turns::TurnItem turn;
  turns::TurnItem nextTurn;

  route.GetCurrentTurn(distance, turn);
  route.GetNextTurn(nextDistance, nextTurn);
  TEST_EQUAL(turn, kTestTurns[0], ());
  TEST_EQUAL(nextTurn, kTestTurns[1], ());

  route.MoveIterator(GetGps(0.5, 1));
  route.GetCurrentTurn(distance, turn);
  route.GetNextTurn(nextDistance, nextTurn);
  TEST_EQUAL(turn, kTestTurns[1], ());
  TEST_EQUAL(nextTurn, kTestTurns[2], ());

  route.MoveIterator(GetGps(1, 1.5));
  route.GetCurrentTurn(distance, turn);
  route.GetNextTurn(nextDistance, nextTurn);
  TEST_EQUAL(turn, kTestTurns[2], ());
  TEST_EQUAL(nextTurn, turns::TurnItem(), ());
}

UNIT_TEST(NextTurnsTest)
{
  Route route("TestRouter");
  route.SetGeometry(kTestGeometry.begin(), kTestGeometry.end());
  vector<turns::TurnItem> turns(kTestTurns);
  route.SetTurnInstructions(move(turns));
  vector<turns::TurnItemDist> turnsDist;

  {
    TEST(route.GetNextTurns(turnsDist), ());
    TEST_EQUAL(turnsDist.size(), 2, ());
    double const firstSegLenM = MercatorBounds::DistanceOnEarth(kTestGeometry[0], kTestGeometry[1]);
    double const secondSegLenM = MercatorBounds::DistanceOnEarth(kTestGeometry[1], kTestGeometry[2]);
    TEST(my::AlmostEqualAbs(turnsDist[0].m_distMeters, firstSegLenM, 0.1), ());
    TEST(my::AlmostEqualAbs(turnsDist[1].m_distMeters, firstSegLenM + secondSegLenM, 0.1), ());
    TEST_EQUAL(turnsDist[0].m_turnItem, kTestTurns[0], ());
    TEST_EQUAL(turnsDist[1].m_turnItem, kTestTurns[1], ());
  }
  {
    double const x = 0.;
    double const y = 0.5;
    route.MoveIterator(GetGps(x, y));
    TEST(route.GetNextTurns(turnsDist), ());
    TEST_EQUAL(turnsDist.size(), 2, ());
    double const firstSegLenM = MercatorBounds::DistanceOnEarth({x, y}, kTestGeometry[1]);
    double const secondSegLenM = MercatorBounds::DistanceOnEarth(kTestGeometry[1], kTestGeometry[2]);
    TEST(my::AlmostEqualAbs(turnsDist[0].m_distMeters, firstSegLenM, 0.1), ());
    TEST(my::AlmostEqualAbs(turnsDist[1].m_distMeters, firstSegLenM + secondSegLenM, 0.1), ());
    TEST_EQUAL(turnsDist[0].m_turnItem, kTestTurns[0], ());
    TEST_EQUAL(turnsDist[1].m_turnItem, kTestTurns[1], ());
  }
  {
    double const x = 1.;
    double const y = 2.5;
    route.MoveIterator(GetGps(x, y));
    TEST(route.GetNextTurns(turnsDist), ());
    TEST_EQUAL(turnsDist.size(), 1, ());
    double const firstSegLenM = MercatorBounds::DistanceOnEarth({x, y}, kTestGeometry[4]);
    TEST(my::AlmostEqualAbs(turnsDist[0].m_distMeters, firstSegLenM, 0.1), ());
    TEST_EQUAL(turnsDist[0].m_turnItem, kTestTurns[2], ());
  }
  {
    double const x = 1.;
    double const y = 3.5;
    route.MoveIterator(GetGps(x, y));
    TEST(route.GetNextTurns(turnsDist), ());
    TEST_EQUAL(turnsDist.size(), 1, ());
  }
}

//   0.0002        *--------*
//                 |        |
//                 |        |
//        Finish   |        |
//   0.0001  *--------------*
//                 |
//                 |
//                 |
//        0        * Start
//           0  0.0001    0.0002
//
UNIT_TEST(SelfIntersectedRouteMatchingTest)
{
  vector<m2::PointD> const kRouteGeometry(
      {{0.0001, 0.0}, {0.0001, 0.0002}, {0.0002, 0.0002}, {0.0002, 0.0001}, {0.0, 0.0001}});
  double constexpr kRoundingErrorMeters = 0.001;

  Route route("TestRouter");
  route.SetGeometry(kRouteGeometry.begin(), kRouteGeometry.end());

  auto const testMachedPos = [&](location::GpsInfo const & pos,
                                 location::GpsInfo const & expectedMatchingPos,
                                 size_t expectedIndexInRoute) {
    location::RouteMatchingInfo routeMatchingInfo;
    route.MoveIterator(pos);
    location::GpsInfo matchedPos = pos;
    route.MatchLocationToRoute(matchedPos, routeMatchingInfo);
    TEST_LESS(MercatorBounds::DistanceOnEarth(
                  m2::PointD(matchedPos.m_latitude, matchedPos.m_longitude),
                  m2::PointD(expectedMatchingPos.m_latitude, expectedMatchingPos.m_longitude)),
              kRoundingErrorMeters, ());
    TEST_EQUAL(routeMatchingInfo.GetIndexInRoute(), expectedIndexInRoute, ());
  };

  // Moving along the route from start point.
  location::GpsInfo const pos1(GetGps(0.0001, 0.0));
  testMachedPos(pos1, pos1, 0 /* expectedIndexInRoute */);
  location::GpsInfo const pos2(GetGps(0.0001, 0.00005));
  testMachedPos(pos2, pos2, 0 /* expectedIndexInRoute */);

  // Moving around the self intersection and checking that position is matched to the first segment of the route.
  location::GpsInfo const selfIntersectionPos(GetGps(0.0001, 0.0001));
  location::GpsInfo const pos3(GetGps(0.00005, 0.0001));
  testMachedPos(pos3, selfIntersectionPos, 0 /* expectedIndexInRoute */);
  location::GpsInfo const pos4(GetGps(0.00015, 0.0001));
  testMachedPos(pos4, selfIntersectionPos, 0 /* expectedIndexInRoute */);

  // Continue moving along the route.
  location::GpsInfo const pos5(GetGps(0.00011, 0.0002));
  testMachedPos(pos5, pos5, 1 /* expectedIndexInRoute */);
  location::GpsInfo const pos6(GetGps(0.0002, 0.00019));
  testMachedPos(pos6, pos6, 2 /* expectedIndexInRoute */);
  location::GpsInfo const pos7(GetGps(0.00019, 0.0001));
  testMachedPos(pos7, pos7, 3 /* expectedIndexInRoute */);

  // Moving around the self intersection and checking that position is matched to the last segment of the route.
  location::GpsInfo const pos8(GetGps(0.0001, 0.00005));
  testMachedPos(pos8, selfIntersectionPos, 3 /* expectedIndexInRoute */);
  location::GpsInfo const pos9(GetGps(0.0001, 0.00015));
  testMachedPos(pos9, selfIntersectionPos, 3 /* expectedIndexInRoute */);
}

UNIT_TEST(RouteNameTest)
{
  Route route("TestRouter");

  route.SetGeometry(kTestGeometry.begin(), kTestGeometry.end());
  vector<turns::TurnItem> turns(kTestTurns);
  route.SetTurnInstructions(move(turns));
  Route::TStreets names(kTestNames);
  route.SetStreetNames(move(names));

  string name;
  route.GetCurrentStreetName(name);
  TEST_EQUAL(name, "Street1", ());

  route.GetStreetNameAfterIdx(0, name);
  TEST_EQUAL(name, "Street1", ());

  route.GetStreetNameAfterIdx(1, name);
  TEST_EQUAL(name, "Street2", ());

  route.GetStreetNameAfterIdx(2, name);
  TEST_EQUAL(name, "Street2", ());

  route.GetStreetNameAfterIdx(3, name);
  TEST_EQUAL(name, "Street2", ());

  route.GetStreetNameAfterIdx(4, name);
  TEST_EQUAL(name, "Street3", ());
}

UNIT_TEST(GetSubrouteInfoTest)
{
  Route route("TestRouter");
  route.SetGeometry(kTestGeometry.begin(), kTestGeometry.end());
  vector<turns::TurnItem> turns(kTestTurns);
  route.SetTurnInstructions(move(turns));
  Route::TTimes times(kTestTimes);
  route.SetSectionTimes(move(times));

  vector<Junction> junctions;
  for (auto const & point : kTestGeometry)
    junctions.emplace_back(point, feature::kDefaultAltitudeMeters);

  vector<RouteSegment> segmentInfo;
  FillSegmentInfo(kTestSegments, junctions, kTestTurns, kTestNames, kTestTimes,
                  nullptr /* trafficStash */, segmentInfo);
  route.SetRouteSegments(move(segmentInfo));
  route.SetSubroteAttrs(vector<Route::SubrouteAttrs>({Route::SubrouteAttrs(
      junctions.front(), junctions.back(), 0 /* beginSegmentIdx */, kTestSegments.size())}));

  TEST_EQUAL(route.GetSubrouteCount(), 1, ());
  vector<RouteSegment> info;
  route.GetSubrouteInfo(0, info);
  TEST_EQUAL(info.size(), 4, ());

  TestSegmentInfo(info[0], turns::TurnDirection::TurnLeft, 1.0 /* distFromBeginningMerc */,
                  traffic::SpeedGroup::Unknown, 5.0/* timeFromBeginningS */);
  TestSegmentInfo(info[1], turns::TurnDirection::TurnRight, 2.0 /* distFromBeginningMerc */,
                  traffic::SpeedGroup::Unknown, 5.0/* timeFromBeginningS */);
  TestSegmentInfo(info[2], turns::TurnDirection::NoTurn, 3.0 /* distFromBeginningMerc */,
                  traffic::SpeedGroup::Unknown, 10.0/* timeFromBeginningS */);
  TestSegmentInfo(info[3], turns::TurnDirection::ReachedYourDestination, 4.0 /* distFromBeginningMerc */,
                  traffic::SpeedGroup::Unknown, 15.0/* timeFromBeginningS */);
}
