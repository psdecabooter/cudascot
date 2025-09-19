#pragma once
#include <memory>

#include "dascot/Types.hpp"

namespace dascot::routing {
std::unique_ptr<dascot::Routing> simAnnealRoute(
    std::unique_ptr<dascot::Mapping> mapping,
    double temperature,
    double cooling_rate,
    double termination_temp);
};