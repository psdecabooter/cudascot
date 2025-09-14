#pragma once
#include <memory>

#include "cudascot/Types.hpp"

namespace cudascot::routing {
std::unique_ptr<cudascot::Routing> simAnnealRoute(
    std::unique_ptr<cudascot::Mapping> mapping,
    double temperature,
    double cooling_rate,
    double termination_temp);
};