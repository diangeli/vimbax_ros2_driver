#pragma once

#include <ctime>
#include <string>

namespace camera_recorder
{

std::string bag_uri(const std::string & output_dir, const std::string & home, const std::tm & local_time);

}  // namespace camera_recorder
