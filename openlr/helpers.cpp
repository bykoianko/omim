#include "openlr/helpers.hpp"

#include "openlr/road_info_getter.hpp"

#include "routing/features_road_graph.hpp"

#include "geometry/mercator.hpp"

#include <sstream>
#include <string>
#include <type_traits>

namespace
{
using namespace openlr;

/// \returns true if |e| conforms |functionalRoadClass| and false otherwise.
/// \param score If returns true |score| fills with an appropriate score.
bool ConformFrc(Graph::Edge const & e, FunctionalRoadClass functionalRoadClass,
                RoadInfoGetter & infoGetter, Score2 & score)
{
  Score2 constexpr kMaxScoreForFrc = 25;

  if (e.IsFake() || functionalRoadClass == FunctionalRoadClass::NotAValue)
    return true;

  auto const hwClass = infoGetter.Get(e.GetFeatureId()).m_hwClass;

  switch (functionalRoadClass)
  {
  case FunctionalRoadClass::FRC0:
    score = kMaxScoreForFrc;
    return hwClass == ftypes::HighwayClass::Trunk;
  case FunctionalRoadClass::FRC1:
    score = kMaxScoreForFrc;
    return hwClass == ftypes::HighwayClass::Trunk || hwClass == ftypes::HighwayClass::Primary;
  case FunctionalRoadClass::FRC2:
  case FunctionalRoadClass::FRC3:
    if (hwClass == ftypes::HighwayClass::Secondary || hwClass == ftypes::HighwayClass::Tertiary)
      score = kMaxScoreForFrc;

    return hwClass == ftypes::HighwayClass::Primary || hwClass == ftypes::HighwayClass::Secondary ||
           hwClass == ftypes::HighwayClass::Tertiary ||
           hwClass == ftypes::HighwayClass::LivingStreet;

  case FunctionalRoadClass::FRC4:
    if (hwClass == ftypes::HighwayClass::LivingStreet || hwClass == ftypes::HighwayClass::Service)
      score = kMaxScoreForFrc;

    return hwClass == ftypes::HighwayClass::Tertiary ||
           hwClass == ftypes::HighwayClass::LivingStreet ||
           hwClass == ftypes::HighwayClass::Service;
  case FunctionalRoadClass::FRC5:
  case FunctionalRoadClass::FRC6:
  case FunctionalRoadClass::FRC7:
    score = kMaxScoreForFrc;
    return hwClass == ftypes::HighwayClass::LivingStreet ||
        hwClass == ftypes::HighwayClass::Service;
  case FunctionalRoadClass::NotAValue:
    UNREACHABLE();
  }
  UNREACHABLE();
}
}  // namespace

namespace openlr
{
bool PointsAreClose(m2::PointD const & p1, m2::PointD const & p2)
{
  double const kMwmRoadCrossingRadiusMeters = routing::GetRoadCrossingRadiusMeters();
  return MercatorBounds::DistanceOnEarth(p1, p2) < kMwmRoadCrossingRadiusMeters;
}

double EdgeLength(Graph::Edge const & e)
{
  return MercatorBounds::DistanceOnEarth(e.GetStartPoint(), e.GetEndPoint());
}

bool EdgesAreAlmostEqual(Graph::Edge const & e1, Graph::Edge const & e2)
{
  // TODO(mgsergio): Do I need to check fields other than points?
  return PointsAreClose(e1.GetStartPoint(), e2.GetStartPoint()) &&
         PointsAreClose(e1.GetEndPoint(), e2.GetEndPoint());
}

std::string LogAs2GisPath(Graph::EdgeVector const & path)
{
  CHECK(!path.empty(), ("Paths should not be empty"));

  std::ostringstream ost;
  ost << "https://2gis.ru/moscow?queryState=";

  auto ll = MercatorBounds::ToLatLon(path.front().GetStartPoint());
  ost << "center%2F" << ll.lon << "%2C" << ll.lat << "%2F";
  ost << "zoom%2F" << 17 << "%2F";
  ost << "ruler%2Fpoints%2F";
  for (auto const & e : path)
  {
    ll = MercatorBounds::ToLatLon(e.GetStartPoint());
    ost << ll.lon << "%20" << ll.lat << "%2C";
  }
  ll = MercatorBounds::ToLatLon(path.back().GetEndPoint());
  ost << ll.lon << "%20" << ll.lat;

  return ost.str();
}

std::string LogAs2GisPath(Graph::Edge const & e) { return LogAs2GisPath(Graph::EdgeVector({e})); }

// @TODO Use fow
bool PassesRestriction(Graph::Edge const & e, FunctionalRoadClass restriction, FormOfWay fow,
                       RoadInfoGetter & infoGetter, Score2 & score)
{
  bool const conform = ConformFrc(e, restriction, infoGetter, score);
  if (!conform)
    return false;

  Score2 constexpr kScoreForFow = 25;
  if (fow == FormOfWay::Roundabout && infoGetter.Get(e.GetFeatureId()).m_isRoundabout)
    score += kScoreForFow;

//  if (fow == FormOfWay::MultipleCarriageway && infoGetter.Get(e.GetFeatureId()).m_oneWay)
//    score += 15;

//  if (fow == FormOfWay::Sliproad && infoGetter.Get(e.GetFeatureId()).m_link)
//    score += kScoreForFow;


  return true;
}

bool ConformLfrcnp(Graph::Edge const & e, FunctionalRoadClass lowestFrcToNextPoint,
                   RoadInfoGetter & infoGetter)
{
  Score2 score;
  return ConformFrc(e, lowestFrcToNextPoint, infoGetter, score);
}
}  // namespace openlr
