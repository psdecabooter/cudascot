#include <fstream>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>

#include "dascot/Mapping.hpp"
#include "dascot/Routing.hpp"
#include "dascot/Types.hpp"

using namespace dascot;

int main(int argc, char *argv[]) {
  // Architecture arch{3, 3, {0, 1}, {2}};

  // Gate g0(Operation::T, 0);
  // Gate g1(Operation::TDG, 1);
  // Gate g2(Operation::CX, 1, 2);

  // Route r0{0, Operation::CX, {0, 1}, {0, 3, 5, 1}};
  // Route r1{1, Operation::T, {0}, {0, 3, 5, 1}};
  // Route r2{2, Operation::TDG, {2}, {0, 3, 5, 1}};

  // Routing routing{
  //     {{0, 0}, {1, 1}},
  //     {{r0, r1, r2}},
  //     arch,
  //     {g0, g1, g2}};
  std::string path = argv[1];
  std::ifstream file(path);
  json j;
  file >> j;
  file.close();
  std::unique_ptr<Circuit> circ = std::make_unique<Circuit>(j.get<Circuit>());
  std::vector<int> q = {0, 1, 2, 3, 4, 5, 6};
  std::unique_ptr<Mapping> mapping = mapping::randomMap(std::move(circ), q);
  // json j_mapping = *mapping;
  // std::cout << j_mapping.dump(4) << "\n";
  std::unique_ptr<Routing> routing =
      routing::simAnnealRoute(std::move(mapping), 10, 0.1, 0.1);
  json j_routing = *routing;

  std::cout << j_routing.dump(4) << "\n";

  return 0;
}