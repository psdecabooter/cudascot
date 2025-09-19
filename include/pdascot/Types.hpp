#pragma once

#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace pdascot {
using json = nlohmann::json;

// Operations
enum class Operation { CX, T, TDG };

inline void to_json(json &j, const Operation &o) {
  switch (o) {
    case Operation::T:
      j = "t";
      break;
    case Operation::TDG:
      j = "tdg";
      break;
    case Operation::CX:
      j = "cx";
      break;
    default:
      throw std::runtime_error("Unknown GateKind");
  }
}

inline void from_json(const json &j, Operation &o) {
  std::string s = j.get<std::string>();
  if (s == "t")
    o = Operation::T;
  else if (s == "tdg")
    o = Operation::TDG;
  else if (s == "cx")
    o = Operation::CX;
  else
    throw std::runtime_error("Invalid gate: " + s);
}

inline bool is_single_qubit(Operation k) {
  return (k == Operation::T || k == Operation::TDG);
}

inline bool is_double_qubit(Operation k) { return (k == Operation::CX); }

// Gates
struct Gate {
  Operation op;
  int q0;
  int q1 = -1;  // only has a value if the gate type != SINGLE_QUBIT

  Gate(Operation k, int qubit) : op(k), q0(qubit) {}
  Gate(Operation k, int qubit0, int qubit1) : op(k), q0(qubit0), q1(qubit1) {}
  Gate() = default;
};

inline void to_json(nlohmann::json &j, const Gate &g) {
  if (is_single_qubit(g.op)) {
    j = json::array({g.q0});
  } else if (is_double_qubit(g.op)) {
    j = json::array({g.q0, g.q1});
  } else {
    throw std::runtime_error("Unsupported gate kind");
  }
}

inline void from_json(const json &j, Gate &g) {
  auto qs = j.get<std::vector<int>>();
  if (qs.size() == 1) {
    g.op = Operation::T;  // or TDG — you’d need some way to know which
    g.q0 = qs[0];
    g.q1 = -1;
  } else if (qs.size() == 2) {
    g.op = Operation::CX;
    g.q0 = qs[0];
    g.q1 = qs[1];
  } else {
    throw std::runtime_error("Invalid gate array size");
  }
}

// Architecture
struct Architecture {
  int height;
  int width;
  std::vector<int> alg_qubits;
  std::vector<int> magic_states;
};

inline void to_json(json &j, const Architecture &a) {
  j = json{{"height", a.height},
           {"width", a.width},
           {"alg_qubits", a.alg_qubits},
           {"magic_states", a.magic_states}};
}

inline void from_json(const json &j, Architecture &a) {
  j.at("height").get_to(a.height);
  j.at("width").get_to(a.width);
  j.at("alg_qubits").get_to(a.alg_qubits);
  j.at("magic_states").get_to(a.magic_states);
}

// Circuit
struct Circuit {
  Architecture arch;
  std::vector<Gate> gates;
};

inline void to_json(json &j, const Circuit &c) {
  j = json{{"arch", c.arch}, {"gates", c.gates}};
}

inline void from_json(const json &j, Circuit &c) {
  j.at("arch").get_to(c.arch);
  j.at("gates").get_to(c.gates);
}

// Map helper function
// Helper function to convert int-keyed map to JSON object with string keys
inline json map_to_json_object(const std::unordered_map<int, int> &map) {
  json obj = json::object();
  for (const auto &[key, value] : map) {
    obj[std::to_string(key)] = value;
  }
  return obj;
}

// Helper function to convert JSON object with string keys back to int-keyed map
inline std::unordered_map<int, int> json_object_to_map(const json &j) {
  std::unordered_map<int, int> map;
  for (auto it = j.begin(); it != j.end(); ++it) {
    int key = std::stoi(it.key());
    int value = it.value().get<int>();
    map[key] = value;
  }
  return map;
}

// Mapping
struct Mapping {
  std::unordered_map<int, int> map;
  Architecture arch;
  std::vector<Gate> gates;

  Mapping() = default;
  Mapping(const Circuit &circ, std::unordered_map<int, int> map)
      : map(map), arch(circ.arch), gates(circ.gates) {}
};

inline void to_json(json &j, const Mapping &m) {
  j = json{
      {"map", map_to_json_object(m.map)}, {"arch", m.arch}, {"gates", m.gates}};
}

inline void from_json(const json &j, Mapping &m) {
  m.map = json_object_to_map(j.at("map"));
  j.at("arch").get_to(m.arch);
  j.at("gates").get_to(m.gates);
}

// Route
struct Route {
  int id;
  Operation op;
  std::vector<int> qubits;
  std::vector<int> path;

  Route() = default;
  Route(int _id,
        Operation _op,
        const std::vector<int> &_qubits,
        const std::vector<int> &_path)
      : id(_id), op(_op), qubits(_qubits), path(_path) {}
};

inline void to_json(json &j, const Route &r) {
  j = json{{"id", r.id}, {"op", r.op}, {"qubits", r.qubits}, {"path", r.path}};
}

inline void from_json(const json &j, Route &r) {
  j.at("id").get_to(r.id);
  j.at("op").get_to(r.op);
  j.at("qubits").get_to(r.qubits);
  j.at("path").get_to(r.path);
}

// Routing
struct Routing {
  std::unordered_map<int, int> map;
  std::vector<std::vector<Route>> steps;
  Architecture arch;
  std::vector<Gate> gates;

  Routing() = default;
  Routing(const Mapping &mapping, std::vector<std::vector<Route>> &&steps)
      : map(mapping.map),
        steps(std::move(steps)),
        arch(mapping.arch),
        gates(mapping.gates) {}
};

inline void to_json(json &j, const Routing &r) {
  j = json::object();
  j["map"] = map_to_json_object(r.map);
  j["steps"] = r.steps;
  j["arch"] = r.arch;
  j["gates"] = r.gates;
}

inline void from_json(const json &j, Routing &r) {
  r.map = json_object_to_map(j.at("map"));
  j.at("steps").get_to(r.steps);
  j.at("arch").get_to(r.arch);
  j.at("gates").get_to(r.gates);
}

}  // namespace pdascot