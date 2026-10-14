#include "fileloader.h"

#include <filesystem>
#include <limits>
#include <stdexcept>

bool FileLoader::open(const string& filename, int columns) {
  static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559,
                "Sensor records require IEEE-754 binary64 doubles");
  if (columns <= 0) throw std::invalid_argument("Input record width must be positive");
  source_path_ = filename;
  columns_ = columns;
  filefp_.open(filename, std::ios::in | std::ios::binary);
  if (!isOpen()) return false;
  const auto bytes = std::filesystem::file_size(filename);
  const auto record_bytes = sizeof(double) * static_cast<size_t>(columns_);
  if (bytes == 0 || bytes % record_bytes != 0)
    throw std::runtime_error("Empty input or incomplete binary record: " + source_path_);
  return true;
}

vector<double> FileLoader::load() {
  vector<double> record(columns_);
  const auto bytes = static_cast<std::streamsize>(record.size() * sizeof(double));
  filefp_.read(reinterpret_cast<char*>(record.data()), bytes);
  if (filefp_.gcount() != bytes || !filefp_)
    throw std::runtime_error("Cannot read a complete input record: " + source_path_);
  return record;
}
