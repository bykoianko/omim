#pragma once

#include "routing/directions_engine.hpp"
#include "routing/num_mwm_id.hpp"

#include <memory>

namespace routing
{

class PedestrianDirectionsEngine : public IDirectionsEngine
{
public:
  PedestrianDirectionsEngine(std::shared_ptr<NumMwmIds> numMwmIds);

  // IDirectionsEngine override:
  void Generate(RoadGraphBase const & graph, std::vector<Junction> const & path,
                my::Cancellable const & cancellable, Route::TTimes & times, Route::TTurns & turns,
                Route::TStreets & streetNames, std::vector<Junction> & routeGeometry,
                std::vector<Segment> & segments) override;

private:
  void CalculateTurns(RoadGraphBase const & graph, std::vector<Edge> const & routeEdges,
                      Route::TTurns & turnsDir, my::Cancellable const & cancellable) const;

  uint32_t const m_typeSteps;
  uint32_t const m_typeLiftGate;
  uint32_t const m_typeGate;
  std::shared_ptr<NumMwmIds> const m_numMwmIds;
};

}  // namespace routing
