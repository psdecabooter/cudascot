#pragma once
#include <memory>
#include <vector>

#include "pdascot/Types.hpp"

namespace pdascot::mapping {
std::unique_ptr<pdascot::Mapping> randomMap(
    std::unique_ptr<pdascot::Circuit> circ,
    const std::vector<int> &usable_qubits);
};