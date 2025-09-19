#pragma once
#include <memory>
#include <vector>

#include "dascot/Types.hpp"

namespace dascot::mapping {
std::unique_ptr<dascot::Mapping> randomMap(
    std::unique_ptr<dascot::Circuit> circ,
    const std::vector<int> &usable_qubits);
};