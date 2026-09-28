#ifndef VIMBAX_CAMERA__STAMP_PAIRER_HPP_
#define VIMBAX_CAMERA__STAMP_PAIRER_HPP_

#include <cstdint>
#include <deque>
#include <vector>

namespace vimbax_camera
{

class StampPairer
{
public:
  struct Window
  {
    uint64_t rgb = 0;
    uint64_t ir = 0;
    uint64_t unmatched_rgb = 0;
    uint64_t unmatched_ir = 0;
    std::vector<int64_t> rgb_minus_ir_ns;
  };

  explicit StampPairer(int64_t max_pair_offset_ns);

  void add_rgb(int64_t stamp_ns);
  void add_ir(int64_t stamp_ns);
  Window take_window();

private:
  void pair();

  int64_t max_pair_offset_ns_;
  std::deque<int64_t> rgb_;
  std::deque<int64_t> ir_;
  Window window_;
};

}  // namespace vimbax_camera

#endif  // VIMBAX_CAMERA__STAMP_PAIRER_HPP_
