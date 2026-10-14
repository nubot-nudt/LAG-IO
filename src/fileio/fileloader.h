#pragma once

#include "filebase.h"

// Fixed-width binary double records shared by the two sensor readers.
class FileLoader : public FileBase {
 public:
  bool open(const string& filename, int columns);
  vector<double> load();

 private:
  string source_path_;
};
