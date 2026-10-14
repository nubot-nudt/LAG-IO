#include <iostream>

#include "config/configuration.h"
#include "graph/system.h"

int main(int argc, char* argv[]) {
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "Usage: lag_io <config.yaml>\n";
    return 0;
  }
  if (argc != 2) {
    std::cerr << "Usage: lag_io <config.yaml>\n";
    return 2;
  }
  try {
    Paras paras = loadConfig(argv[1]);
    System system(paras);
    system.run();
  } catch (const std::exception& error) {
    std::cerr << "lag_io: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
