#pragma once
#include <memory>
#include <vector>

#include "cudascot/Types.hpp"

namespace cudascot::mapping {
std::unique_ptr<cudascot::Mapping> randomMap(
    std::unique_ptr<cudascot::Circuit> circ,
    const std::vector<int> &usable_qubits);
};