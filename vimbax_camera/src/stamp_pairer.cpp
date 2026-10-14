#include <utility>

#include <vimbax_camera/stamp_pairer.hpp>

namespace vimbax_camera
{

StampPairer::StampPairer(int64_t max_pair_offset_ns)
: max_pair_offset_ns_{max_pair_offset_ns}
{
}

void StampPairer::add_rgb(int64_t stamp_ns)
{
  rgb_.push_back(stamp_ns);
  ++window_.rgb;
  pair();
}

void StampPairer::add_ir(int64_t stamp_ns)
{
  ir_.push_back(stamp_ns);
  ++window_.ir;
  pair();
}

StampPairer::Window StampPairer::take_window()
{
  return std::exchange(window_, Window{});
}

void StampPairer::pair()
{
  while (!rgb_.empty() && !ir_.empty()) {
    auto const rgb_minus_ir = rgb_.front() - ir_.front();
    if (rgb_minus_ir >= -max_pair_offset_ns_ && rgb_minus_ir <= max_pair_offset_ns_) {
      window_.rgb_minus_ir_ns.push_back(rgb_minus_ir);
      rgb_.pop_front();
      ir_.pop_front();
    } else if (rgb_minus_ir < 0) {
      rgb_.pop_front();
      ++window_.unmatched_rgb;
    } else {
      ir_.pop_front();
      ++window_.unmatched_ir;
    }
  }
}

}  // namespace vimbax_camera
