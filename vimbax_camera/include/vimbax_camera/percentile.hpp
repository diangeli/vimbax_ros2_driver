#ifndef VIMBAX_CAMERA__PERCENTILE_HPP_
#define VIMBAX_CAMERA__PERCENTILE_HPP_

#include <algorithm>
#include <vector>

namespace vimbax_camera
{

inline double percentile(std::vector<double> values, double q)
{
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  return values[static_cast<std::size_t>(q * (values.size() - 1) + 0.5)];
}

}  // namespace vimbax_camera

#endif  // VIMBAX_CAMERA__PERCENTILE_HPP_
