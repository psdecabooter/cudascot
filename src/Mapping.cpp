#include <algorithm>
#include <memory>
#include <random>
#include <unordered_map>
#include <vector>

#include "pdascot/Types.hpp"

namespace pdascot::mapping {
std::unique_ptr<pdascot::Mapping> randomMap(
    std::unique_ptr<pdascot::Circuit> circ,
    const std::vector<int> &usable_qubits) {
  if (usable_qubits.size() > circ->arch.alg_qubits.size()) {
    throw std::runtime_error("Not enough qubit locations for mapping");
  }
  std::unordered_map<int, int> map;
  // Random
  std::random_device rd;
  std::mt19937 g(rd());
  // Copy then shuffle
  std::vector<int> shuffled_qubits = usable_qubits;
  std::shuffle(shuffled_qubits.begin(), shuffled_qubits.end(), g);
  std::vector<int> shuffled_locations = circ->arch.alg_qubits;
  std::shuffle(shuffled_locations.begin(), shuffled_locations.end(), g);
  // Assign shuffled qubits
  for (size_t i = 0; i < shuffled_qubits.size(); ++i) {
    map[shuffled_qubits.at(i)] = shuffled_locations.at(i);
  }

  auto mapping = std::make_unique<pdascot::Mapping>(*circ, map);
  return mapping;
}
};  // namespace pdascot::mapping