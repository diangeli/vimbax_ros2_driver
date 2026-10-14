#include "camera_recorder/bag_uri.hpp"

#include <algorithm>
#include <filesystem>

namespace camera_recorder
{

std::string bag_uri(const std::string & output_dir, const std::string & home, const std::tm & local_time)
{
  std::filesystem::path dir = output_dir;
  if (output_dir == "~" || output_dir.rfind("~/", 0) == 0) {
    dir = std::filesystem::path(home) / output_dir.substr(std::min<std::size_t>(2, output_dir.size()));
  }
  char name[32];
  std::strftime(name, sizeof(name), "camera_%Y%m%d_%H%M%S", &local_time);
  return (dir / name).lexically_normal().string();
}

}  // namespace camera_recorder
