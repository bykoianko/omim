#pragma once

#include "geometry/point2d.hpp"

#include "base/assert.hpp"

#include <cstdint>
#include <functional>
#include <map>
#include <string>

namespace routing
{
class Route;

/// Routing possible statuses enumeration.
/// \warning  this enum has JNI mirror!
/// \see android/src/com/mapswithme/maps/routing/ResultCodesHelper.java
// TODO(bykoianko): Items become obsolete now should be removed from the enum.
enum class RouterResultCode
{
  NoError = 0,
  Cancelled = 1,
  NoCurrentPosition = 2,
  InconsistentMWMandRoute = 3,
  RouteFileNotExist = 4,
  StartPointNotFound = 5,
  EndPointNotFound = 6,
  PointsInDifferentMWM = 7,
  RouteNotFound = 8,
  NeedMoreMaps = 9,
  InternalError = 10,
  FileTooOld = 11,
  IntermediatePointNotFound = 12,
  TransitRouteNotFoundNoNetwork = 13,
  TransitRouteNotFoundTooLongPedestrian = 14,
  RouteNotFoundRedressRouteError = 15,
};

using CheckpointCallback = std::function<void(size_t passedCheckpointIdx)>;
using ProgressCallback = std::function<void(float)>;
// @TODO(bykoianko) ReadyCallback and ReadyCallbackOwnership callbacks should be gathered
// to one with the following signature:
// std::function<void(std::unique_ptr<Route>, RouterResultCode)>
// That means calling ReadyCallback means passing ownership of ready instance of Route.
using ReadyCallback = std::function<void(Route const &, RouterResultCode)>;
using ReadyCallbackOwnership = std::function<void(Route &, RouterResultCode)>;
using RouteCallback = std::function<void(Route const &)>;
using RoutingStatisticsCallback = std::function<void(std::map<std::string, std::string> const &)>;
using PointCheckCallback = std::function<void(m2::PointD const &)>;

inline std::string DebugPrint(RouterResultCode code)
{
  switch (code)
  {
  case RouterResultCode::NoError: return "NoError";
  case RouterResultCode::Cancelled: return "Cancelled";
  case RouterResultCode::NoCurrentPosition: return "NoCurrentPosition";
  case RouterResultCode::InconsistentMWMandRoute: return "InconsistentMWMandRoute";
  case RouterResultCode::RouteFileNotExist: return "RouteFileNotExist";
  case RouterResultCode::StartPointNotFound: return "StartPointNotFound";
  case RouterResultCode::EndPointNotFound: return "EndPointNotFound";
  case RouterResultCode::PointsInDifferentMWM: return "PointsInDifferentMWM";
  case RouterResultCode::RouteNotFound: return "RouteNotFound";
  case RouterResultCode::InternalError: return "InternalError";
  case RouterResultCode::NeedMoreMaps: return "NeedMoreMaps";
  case RouterResultCode::FileTooOld: return "FileTooOld";
  case RouterResultCode::IntermediatePointNotFound: return "IntermediatePointNotFound";
  case RouterResultCode::TransitRouteNotFoundNoNetwork: return "TransitRouteNotFoundNoNetwork";
  case RouterResultCode::TransitRouteNotFoundTooLongPedestrian: return "TransitRouteNotFoundTooLongPedestrian";
  case RouterResultCode::RouteNotFoundRedressRouteError: return "RouteNotFoundRedressRouteError";
  }

  std::string const result = "Unknown RouterResultCode:" + std::to_string(static_cast<int>(code));
  ASSERT(false, (result));
  return result;
}
}  // namespace routing
