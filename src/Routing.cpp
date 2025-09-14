#include <algorithm>
#include <climits>
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <queue>
#include <random>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "cudascot/Types.hpp"

/**
 * ChatGPT astar grid search
 */
std::vector<int> astar_grid(const std::vector<std::vector<int>> &grid,
                            int start,
                            int goal) {
  int H = grid.size();
  int W = grid[0].size();
  std::vector<double> g_score(H * W, std::numeric_limits<double>::infinity());
  std::vector<int> came_from(H * W, -1);

  auto heuristic = [&](int v) {
    int x = v % W, y = v / W;
    int gx = goal % W, gy = goal / W;
    return std::abs(x - gx) + std::abs(y - gy);
  };

  using PQ = std::pair<double, int>;
  std::priority_queue<PQ, std::vector<PQ>, std::greater<PQ>> open_set;
  open_set.emplace(heuristic(start), start);
  g_score[start] = 0;

  while (!open_set.empty()) {
    int current = open_set.top().second;
    open_set.pop();

    if (current == goal) {
      // reconstruct path
      std::vector<int> path;
      for (int v = goal; v != -1; v = came_from[v]) path.push_back(v);
      std::reverse(path.begin(), path.end());
      return path;
    }

    int cx = current % W, cy = current / W;
    std::vector<std::pair<int, int>> neighbors = {
        {cx + 1, cy}, {cx - 1, cy}, {cx, cy + 1}, {cx, cy - 1}};

    for (auto [nx, ny] : neighbors) {
      if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
      if (grid[ny][nx] == -1) continue;  // blocked

      int neighbor = ny * W + nx;
      double tentative_g = g_score[current] + 1;
      if (tentative_g < g_score[neighbor]) {
        came_from[neighbor] = current;
        g_score[neighbor] = tentative_g;
        open_set.emplace(tentative_g + heuristic(neighbor), neighbor);
      }
    }
  }
  return {};  // no path found
}

std::vector<int> verticalNeighbors(int loc, int width, int height) {
  std::vector<int> neighbors;
  int down = loc + width;
  int up = loc - width;
  if (loc / width != 0) {
    neighbors.push_back(up);
  }
  if (loc / width != height - 1) {
    neighbors.push_back(down);
  }
  return neighbors;
}

std::vector<int> horizontalNeighbors(int loc, int width) {
  std::vector<int> neighbors;
  int left = loc - 1;
  int right = loc + 1;
  if (loc % width != 0) neighbors.push_back(left);
  if (loc % width != width - 1) neighbors.push_back(right);
  return neighbors;
}

inline std::pair<int, int> locToXY(int loc, int width) {
  return {loc % width, loc / width};
}

/**
 * @brief Function for routing a gate
 *
 * @details Routes a gate, returns the route, modifies the to_remove
 */
std::optional<cudascot::Route> routeGate(int gate_id,
                                         const cudascot::Gate &gate,
                                         const cudascot::Mapping &mapping,
                                         std::unordered_set<int> &to_remove) {
  std::vector<std::vector<int>> grid(mapping.arch.height,
                                     std::vector<int>(mapping.arch.width, 0));
  for (int loc : to_remove) {
    int y = loc / mapping.arch.width;
    int x = loc % mapping.arch.width;
    grid[y][x] = -1;
  }
  int shortest_path_len = INT_MAX;
  std::vector<int> shortest_path;
  std::vector<std::pair<int, int>> pairs;
  if (cudascot::is_double_qubit(gate.op)) {
    // Source q0 vertical neighbors, destination q1 horizontal neighbors
    for (int vn : verticalNeighbors(
             mapping.map.at(gate.q0), mapping.arch.width, mapping.arch.height))
      for (int hn :
           horizontalNeighbors(mapping.map.at(gate.q1), mapping.arch.width))
        pairs.push_back(std::pair(vn, hn));
  } else {
    // Source q0 vertical neighbors, destination any magic state horizontal
    // neighbor
    int target = mapping.map.at(gate.q0);
    std::vector<int> sorted_msf = mapping.arch.magic_states;
    // Sorting magic state faces by manhattan distance to q0 face
    std::sort(sorted_msf.begin(), sorted_msf.end(), [&](int m1, int m2) {
      auto manhattan = [&](int a) {
        int ax = a % mapping.arch.width;
        int ay = a / mapping.arch.width;
        int tx = target % mapping.arch.width;
        int ty = target / mapping.arch.width;
        return std::abs(ax - tx) + std::abs(ay - ty);
      };
      return manhattan(m1) < manhattan(m2);
    });
    for (int magic_state : sorted_msf)
      for (int vn : verticalNeighbors(mapping.map.at(gate.q0),
                                      mapping.arch.width,
                                      mapping.arch.height))
        for (int hn : horizontalNeighbors(magic_state, mapping.arch.width))
          pairs.push_back(std::pair(vn, hn));
  }
  // Filter out pairs which have locations inside of to_remove
  pairs.erase(
      std::remove_if(pairs.begin(),
                     pairs.end(),
                     [&](const std::pair<int, int> &p) {
                       return (to_remove.find(p.first) != to_remove.end()) ||
                              (to_remove.find(p.second) != to_remove.end());
                     }),
      pairs.end());
  for (auto [s, t] : pairs) {
    // RUN A* here
    std::vector<int> path = astar_grid(grid, s, t);
    int distance = INT_MAX;
    if (!path.empty()) {
      distance = path.size();
    }
    if (distance < shortest_path_len) {
      shortest_path_len = distance;
      shortest_path = std::move(path);
    }
  }
  if (shortest_path_len == INT_MAX) {
    return std::nullopt;
  }
  // update to_remove
  for (int loc : shortest_path) {
    // std::cout << loc << ' ';
    to_remove.insert(loc);
  }
  // std::cout << '\n';
  std::vector<int> qubits;
  qubits.push_back(gate.q0);
  if (cudascot::is_double_qubit(gate.op)) {
    qubits.push_back(gate.q1);
  }
  return cudascot::Route(
      gate_id, gate.op, std::move(qubits), std::move(shortest_path));
}

std::unordered_set<int> initializeToRemove(const cudascot::Mapping &map) {
  std::unordered_set<int> to_remove;
  for (const auto &[q, loc] : map.map) to_remove.insert(loc);
  for (int loc : map.arch.magic_states) to_remove.insert(loc);
  return to_remove;
}

std::vector<cudascot::Route> tryOrder(
    const std::vector<int> &order,
    const std::map<int, cudascot::Gate> &executable_gates,
    const cudascot::Mapping &map) {
  std::vector<cudascot::Route> step;
  std::unordered_set<int> to_remove = initializeToRemove(map);
  // std::cout << "order\n";
  for (int i = 0; i < executable_gates.size(); i++) {
    int id = order.at(i);
    cudascot::Gate gate = executable_gates.at(id);
    // Route the gate, mutates to_remove
    std::optional<cudascot::Route> route = routeGate(id, gate, map, to_remove);
    if (!route.has_value()) continue;
    // std::cout << to_remove.size() << '\n';
    // std::cout << route->id << '\n';
    // std::cin.get();
    step.push_back(*route);
  }
  return step;
}

int criticalityFast(const std::vector<cudascot::Route> &step,
                    const std::vector<int> &gate_crits) {
  int paths = 0;
  for (const cudascot::Route &r : step) {
    paths += gate_crits.at(r.id);
  }
  return paths;
}

/**
 * Differences from dascot:
 * I do not try to swap cnots and t gates, because im not sure dascot even does
 * that! Since the order is randomized, the destinction between the two does not
 * change.
 * I am only implementing the criticality heuristic.
 */
std::tuple<std::vector<cudascot::Route>, int> bestRealizableSetFound(
    const std::map<int, cudascot::Gate> &id_gates,
    const std::map<int, cudascot::Gate> &executable_gates,
    const cudascot::Mapping &mapping,
    const std::vector<int> &gate_crits,
    double temperature,
    double cooling_rate,
    double termination_temp) {
  std::vector<int> single_ids;
  std::vector<int> double_ids;
  std::vector<int> best_order;
  for (const auto &[id, gate] : executable_gates) {
    if (cudascot::is_double_qubit(gate.op)) {
      double_ids.push_back(id);
    } else {
      single_ids.push_back(id);
    }
    best_order.push_back(id);
  }
  // Default best order is random
  std::shuffle(
      best_order.begin(), best_order.end(), std::default_random_engine());
  std::vector<cudascot::Route> best_step =
      tryOrder(best_order, executable_gates, mapping);
  std::vector<int> current_order = best_order;
  std::vector<cudascot::Route> current_step = best_step;
  int orders_tried = 1;
  // Trivial case
  if (executable_gates.size() < 2) {
    return {best_step, 1};
  }
  // Brute force case
  if (double_ids.size() < 5 and single_ids.size() < 5 and cooling_rate != 1) {
    // Not implementing for now
  }
  // Random number generator for swaps
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, executable_gates.size() - 1);
  // Simulated Annealing
  while (temperature > termination_temp) {
    std::vector<int> new_order = current_order;
    // Swap two gates in the order
    int index1 = dis(gen);
    int index2;
    do {
      index2 = dis(gen);
    } while (index2 == index1);
    std::swap(new_order[index1], new_order[index2]);
    std::vector<cudascot::Route> new_step =
        tryOrder(new_order, executable_gates, mapping);
    orders_tried++;
    int delta_curr = criticalityFast(current_step, gate_crits) -
                     criticalityFast(new_step, gate_crits);
    int delta_best = criticalityFast(best_step, gate_crits) -
                     criticalityFast(new_step, gate_crits);
    if (delta_curr < 0 || std::exp(-delta_curr / temperature)) {
      current_order = new_order;
      current_step = new_step;
    }
    if (delta_best < 0) {
      best_order = new_order;
      best_step = new_step;
    }
    // cool down
    temperature *= (1.0 - cooling_rate);
  }

  return {best_step, orders_tried};
}

std::tuple<std::map<int, cudascot::Gate>, std::map<int, cudascot::Gate>>
executableSubset(const std::map<int, cudascot::Gate> &id_gates) {
  std::map<int, cudascot::Gate> executable;
  std::map<int, cudascot::Gate> remaining;
  std::unordered_set<int> blocked_qubits;
  for (const auto &[id, gate] : id_gates) {
    bool not_blocked = (blocked_qubits.find(gate.q0) == blocked_qubits.end()) &&
                       (cudascot::is_single_qubit(gate.op) ||
                        (blocked_qubits.find(gate.q1) == blocked_qubits.end()));
    if (not_blocked) {
      executable[id] = gate;
    } else {
      remaining[id] = gate;
    }
    blocked_qubits.insert(gate.q0);
    if (cudascot::is_double_qubit(gate.op)) blocked_qubits.insert(gate.q1);
  }
  return {executable, remaining};
}

std::unordered_map<int, int> depthByQubit(
    int i, const std::vector<cudascot::Gate> &gates) {
  std::unordered_map<int, int> qubit_depths;
  std::unordered_set<int> touched_qubits;
  // Add first gate
  touched_qubits.insert(gates[i].q0);
  if (cudascot::is_double_qubit(gates[i].op)) {
    touched_qubits.insert(gates[i].q1);
  }
  for (; i < gates.size(); i++) {
    cudascot::Gate gate = gates[i];
    if ((touched_qubits.find(gate.q0) != touched_qubits.end()) ||
        (cudascot::is_double_qubit(gate.op) &&
         (touched_qubits.find(gate.q1) != touched_qubits.end()))) {
      touched_qubits.insert(gate.q0);
      if (cudascot::is_double_qubit(gate.op)) {
        touched_qubits.insert(gate.q1);
      }
      int deepest = 0;
      if (qubit_depths.find(gate.q0) != qubit_depths.end()) {
        deepest = qubit_depths[gate.q0] + 1;
      }
      if (cudascot::is_double_qubit(gate.op)) {
        if (qubit_depths.find(gate.q1) != qubit_depths.end()) {
          deepest = std::max(deepest, qubit_depths[gate.q1] + 1);
        }
        qubit_depths[gate.q1] = deepest;
      }
      qubit_depths[gate.q0] = deepest;
    }
  }
  return qubit_depths;
}

std::vector<int> findGateCrits(const std::vector<cudascot::Gate> &gates) {
  std::vector<int> gate_crits(gates.size(), 0);
  for (int i = 0; i < gates.size(); i++) {
    cudascot::Gate gate = gates[i];
    std::unordered_map<int, int> qubit_depths = depthByQubit(i, gates);
    int crit_path = 0;
    for (const auto &kv : qubit_depths) {
      crit_path = std::max(crit_path, kv.second);
    }
    gate_crits[i] = crit_path;
  }
  return gate_crits;
}

std::vector<int> findGateDepths(const std::vector<cudascot::Gate> &gates) {
  std::unordered_map<int, int> qubit_depths;
  std::vector<int> gate_depths(gates.size(), 0);
  for (int i = 0; i < gates.size(); i++) {
    cudascot::Gate gate = gates[i];
    int deepest = 0;
    if (qubit_depths.find(gate.q0) != qubit_depths.end()) {
      deepest = qubit_depths[gate.q0] + 1;
    }
    if (cudascot::is_double_qubit(gate.op)) {
      if (qubit_depths.find(gate.q1) != qubit_depths.end()) {
        deepest = std::max(deepest, qubit_depths[gate.q1] + 1);
      }
      qubit_depths[gate.q1] = deepest;
    }
    qubit_depths[gate.q0] = deepest;
    gate_depths[i] = deepest;
  }
  return gate_depths;
}

namespace cudascot::routing {
std::unique_ptr<cudascot::Routing> simAnnealRoute(
    std::unique_ptr<cudascot::Mapping> mapping,
    double temperature,
    double cooling_rate,
    double termination_temp) {
  std::vector<std::vector<cudascot::Route>> time_steps;
  std::vector<int> gate_crits = findGateCrits(mapping->gates);
  std::map<int, cudascot::Gate> id_gates;
  for (size_t i = 0; i < mapping->gates.size(); i++) {
    id_gates[i] = mapping->gates[i];
  }
  while (!id_gates.empty()) {
    // std::cout << "huh\n" << std::endl;
    auto [executable_gates, remaining_gates] = executableSubset(id_gates);
    // std::cout << executable_gates.size();
    auto [step, tried] = bestRealizableSetFound(id_gates,
                                                executable_gates,
                                                *mapping,
                                                gate_crits,
                                                temperature,
                                                cooling_rate,
                                                termination_temp);
    time_steps.push_back(step);
    std::unordered_set<int> routed_ids;
    for (const cudascot::Route &r : step) {
      routed_ids.insert(r.id);
    }
    std::map<int, cudascot::Gate> not_executed;
    for (const auto &[id, gate] : executable_gates)
      if (routed_ids.find(id) == routed_ids.end()) {
        not_executed[id] = gate;
      }
    id_gates = not_executed;
    id_gates.merge(remaining_gates);
  }
  return std::make_unique<cudascot::Routing>(*mapping, std::move(time_steps));
}
}  // namespace cudascot::routing