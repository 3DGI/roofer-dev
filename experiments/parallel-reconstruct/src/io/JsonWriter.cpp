#include "JsonWriter.h"

#include <thread>

const auto SLEEP_PER_FILE = std::chrono::milliseconds(5);

void JsonWriter::write(Points& models, const std::filesystem::path& path) {
  std::ofstream o(path);

  auto data = json::array();
  for (auto coordinate : models.x) {
    auto pt = json::array();
    pt.push_back(coordinate);
    pt.push_back(coordinate);
    pt.push_back(coordinate);
    data.push_back(pt);
  }
  o << data << '\n';

  // Simulate slow writes
  std::this_thread::sleep_for(SLEEP_PER_FILE);
}
