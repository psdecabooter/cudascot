#pragma once
#include <memory>

#include "pdascot/Types.hpp"

namespace pdascot::routing {
std::unique_ptr<pdascot::Routing> simAnnealRoute(
    std::unique_ptr<pdascot::Mapping> mapping,
    double temperature,
    double cooling_rate,
    double termination_temp);
};