#pragma once

#include "routing/road_graph.hpp"
#include "routing/routing_settings.hpp"
#include "routing/segment.hpp"
#include "routing/turns.hpp"

#include "traffic/speed_groups.hpp"

#include "indexer/feature_altitude.hpp"

#include "geometry/polyline2d.hpp"

#include "base/followed_polyline.hpp"

#include "std/vector.hpp"
#include "std/set.hpp"
#include "std/string.hpp"

#include <limits>
#include <string>

namespace location
{
  class GpsInfo;
  class RouteMatchingInfo;
}

namespace routing
{

class Route
{
public:
  using TTurns = vector<turns::TurnItem>;
  using TTimeItem = pair<uint32_t, double>;
  using TTimes = vector<TTimeItem>;
  using TStreetItem = pair<uint32_t, string>;
  using TStreets = vector<TStreetItem>;

  static uint64_t constexpr kInvalidId = std::numeric_limits<uint64_t>::max();

  /// \brief The route is composed of one or several subroutes. Every subroute is composed of segments.
  /// For every Segment is kept some attributes in the structure SegmentInfo.
  struct SegmentInfo
  {
    SegmentInfo(Segment const & segment, turns::TurnItem const & turn, Junction const & junction,
                std::string const & streetName, double distFromBeginningMeters, double distFromBeginningMerc,
                double timeFromBeginningS, traffic::SpeedGroup traffic)
      : m_segment(segment)
      , m_turn(turn)
      , m_junction(junction)
      , m_streetName(streetName)
      , m_distFromBeginningMeters(distFromBeginningMeters)
      , m_distFromBeginningMerc(distFromBeginningMerc)
      , m_timeFromBeginningS(timeFromBeginningS)
      , m_traffic(traffic)
    {
    }

    Segment m_segment;
    /// Turn (maneuver) information for the turn next to the |m_segment| if any.
    /// If not |m_turn::m_turn| is equal to TurnDirection::NoTurn.
    turns::TurnItem m_turn;
    Junction m_junction; ///< The front point of segment.
    /// Street name of |m_segment| if any. Otherwise |m_streetName| is empty.
    std::string m_streetName;
    /// Distance from the route beginning to the farthest end of |m_segment| in meters.
    double m_distFromBeginningMeters = 0.0;
    /// Distance from the route beginning to the farthest end of |m_segment| in mercator.
    double m_distFromBeginningMerc = 0.0;
    /// ETA from the route beginning in seconds to reach the farthest from the route beginning end of |m_segment|.
    double m_timeFromBeginningS = 0.0;
    traffic::SpeedGroup m_traffic = traffic::SpeedGroup::Unknown;
  };

  /// \brief For every subroute some attributes are kept the following stucture.
  struct SubrouteSettings
  {
    SubrouteSettings(RoutingSettings const & routingSettings, string const & router, uint64_t id)
      : m_routingSettings(routingSettings), m_router(router), m_id(id)
    {
    }

    RoutingSettings m_routingSettings;
    string m_router;
    /// Some subsystems (for example drape) which is used Route class need to have an id of any subroute.
    /// This subsystems may set the id and then use it. The id is kept in |m_id|.
    uint64_t m_id = kInvalidId;
  };

  explicit Route(string const & router)
    : m_router(router), m_routingSettings(GetCarRoutingSettings()) {}

  template <class TIter>
  Route(string const & router, TIter beg, TIter end)
    : m_router(router), m_routingSettings(GetCarRoutingSettings()), m_poly(beg, end)
  {
    Update();
  }

  Route(string const & router, vector<m2::PointD> const & points, string const & name = string());

  void Swap(Route & rhs);

  template <class TIter> void SetGeometry(TIter beg, TIter end)
  {
    if (beg == end)
      FollowedPolyline().Swap(m_poly);
    else
      FollowedPolyline(beg, end).Swap(m_poly);
    Update();
  }

  inline void SetTurnInstructions(TTurns && v) { m_turns = move(v); }
  inline void SetSectionTimes(TTimes && v) { m_times = move(v); }
  inline void SetStreetNames(TStreets && v) { m_streets = move(v); }
  inline void SetAltitudes(feature::TAltitudes && v) { m_altitudes = move(v); }
  inline void SetTraffic(vector<traffic::SpeedGroup> && v) { m_traffic = move(v); }
  /// \brief Glues all |route| attributes to |this| except for |m_altitudes|.
  // @TODO In the future this method should append |m_altitudes| as well.
  // It's not implemented now because it's not easy to do it and it'll not be used in
  // the short future. The problem is routes generated by osrm have empty |m_altitudes|.
  // So |m_altitudes| should be filled (with correnct or default values) on osrm route
  // reconstruction stage to be added with this method. On the other
  // hand it seems this method'll not be not used for bicycle and pedestrian routing
  // in short future. And altitude is not used for car routing.
  void AppendRoute(Route const & route);

  uint32_t GetTotalTimeSec() const;
  uint32_t GetCurrentTimeToEndSec() const;

  FollowedPolyline const & GetFollowedPolyline() const { return m_poly; }

  string const & GetRouterId() const { return m_router; }
  m2::PolylineD const & GetPoly() const { return m_poly.GetPolyline(); }
  TTurns const & GetTurns() const { return m_turns; }
  feature::TAltitudes const & GetAltitudes() const { return m_altitudes; }
  vector<traffic::SpeedGroup> const & GetTraffic() const { return m_traffic; }
  vector<double> const & GetSegDistanceM() const { return m_poly.GetSegDistanceM(); }
  /// \returns distance to all turns in mercator.
  void GetTurnsDistances(vector<double> & distances) const;
  bool IsValid() const { return (m_poly.GetPolyline().GetSize() > 1); }

  double GetTotalDistanceMeters() const;
  double GetCurrentDistanceFromBeginMeters() const;
  double GetCurrentDistanceToEndMeters() const;
  double GetMercatorDistanceFromBegin() const;

  /// \brief GetCurrentTurn returns information about the nearest turn.
  /// \param distanceToTurnMeters is a distance from current position to the nearest turn.
  /// \param turn is information about the nearest turn.
  bool GetCurrentTurn(double & distanceToTurnMeters, turns::TurnItem & turn) const;

  /// \brief Returns a name of a street where the user rides at this moment.
  void GetCurrentStreetName(string &) const;

  /// \brief Returns a name of a street next to idx point of the path. Function avoids short unnamed links.
  void GetStreetNameAfterIdx(uint32_t idx, string &) const;

  /// @return true if GetNextTurn() returns a valid result in parameters, false otherwise.
  /// \param distanceToTurnMeters is a distance from current position to the second turn.
  /// \param turn is information about the second turn.
  /// @return true if its parameters are filled with correct result.
  /// \note All parameters are filled while a GetNextTurn function call.
  bool GetNextTurn(double & distanceToTurnMeters, turns::TurnItem & turn) const;
  /// \brief Extract information about zero, one or two nearest turns depending on current position.
  /// @return true if its parameter is filled with correct result. (At least with one element.)
  bool GetNextTurns(vector<turns::TurnItemDist> & turns) const;

  void GetCurrentDirectionPoint(m2::PointD & pt) const;

  /// @return true  If position was updated successfully (projection within gps error radius).
  bool MoveIterator(location::GpsInfo const & info) const;

  void MatchLocationToRoute(location::GpsInfo & location, location::RouteMatchingInfo & routeMatchingInfo) const;

  bool IsCurrentOnEnd() const;

  /// Add country name if we have no country filename to make route
  void AddAbsentCountry(string const & name);

  /// Get absent file list of a routing files for shortest path finding
  set<string> const & GetAbsentCountries() const { return m_absentCountries; }

  inline void SetRoutingSettings(RoutingSettings const & routingSettings)
  {
    m_routingSettings = routingSettings;
    Update();
  }

  // Subroute interface.
  /// \returns Number of subroutes.
  /// \note Intermediate points separate a route into several subroutes.
  size_t GetSubrouteCount() const;

  /// \brief Fills |info| with full subroute information.
  /// \param subrouteInx zero base number of subroute. |subrouteInx| should be less than GetSubrouteCount();
  /// \note |info| is a segment oriented route. Size of |info| is equal to number of points in |m_poly| - 1.
  /// Class Route is a point oriented route. While this conversion some attributes of zero point will be lost.
  /// It happens with zero turn for example.
  /// \note It's a fake implementation for single subroute which is equal to route without any
  /// intermediate points.
  /// Note. SegmentInfo::m_segment is filled with default Segment instance.
  /// Note. SegmentInfo::m_streetName is filled with an empty string.
  void GetSubrouteInfo(size_t subrouteInx, vector<SegmentInfo> & info) const;

  /// \returns Subroute settings by |subRouteInx|.
  // @TODO(bykoianko) This method should return SubrouteSettings by reference. Now it returns by value
  // because of fake implementation.
  SubrouteSettings const GetSubrouteSettings(size_t subrouteInx) const;

  /// \brief Sets subroute id by |subRouteInx|.
  /// \note |subrouteId| is a permanent id of a subroute. This id can be used to address to a subroute
  /// after the route is removed.
  void SetSubrouteId(size_t subrouteInx, uint64_t subrouteId);

private:
  friend string DebugPrint(Route const & r);

  /// Call this fucnction when geometry have changed.
  void Update();
  double GetPolySegAngle(size_t ind) const;
  TTurns::const_iterator GetCurrentTurn() const;
  TStreets::const_iterator GetCurrentStreetNameIterAfter(FollowedPolyline::Iter iter) const;
  void AppendTraffic(Route const & route);

  string m_router;
  RoutingSettings m_routingSettings;
  string m_name;

  FollowedPolyline m_poly;
  FollowedPolyline m_simplifiedPoly;

  set<string> m_absentCountries;

  TTurns m_turns;
  TTimes m_times;
  TStreets m_streets;
  feature::TAltitudes m_altitudes;
  vector<traffic::SpeedGroup> m_traffic;

  mutable double m_currentTime;

  // Subroute
  uint64_t m_subrouteId = kInvalidId;
};

} // namespace routing
