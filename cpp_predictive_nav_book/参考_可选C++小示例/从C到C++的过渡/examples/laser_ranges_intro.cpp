// 第 1 章示例：不依赖 ROS 的激光距离预处理。
// 编译命令见上一层 README.md。

#include <cmath>       // std::isfinite
#include <cstddef>     // std::size_t
#include <iomanip>     // std::setprecision
#include <iostream>    // std::cout
#include <limits>      // std::numeric_limits
#include <stdexcept>   // std::invalid_argument
#include <vector>      // std::vector

namespace predictive_nav::tutorial {

// 一帧扫描经过过滤后的结果。struct 默认成员是 public，适合纯数据。
struct ScanSummary {
  std::size_t valid_count{0};
  double nearest_range{std::numeric_limits<double>::infinity()};
  std::vector<double> valid_ranges;
};

// RangeFilter 把“数据”和“处理数据的方法”放在一起。
class RangeFilter {
 public:
  // 创建对象时检查规则是否合理，然后保存两条规则。
  RangeFilter(double min_range, double max_range)
      : min_range_(min_range), max_range_(max_range) {
    if (!(min_range_ >= 0.0 && min_range_ < max_range_)) {
      throw std::invalid_argument("min_range 必须非负且小于 max_range");
    }
  }

  // const 说明：这个函数不会修改 min_range_ 或 max_range_。
  // const vector& 说明：只读地使用调用者的数据，不复制整个 vector。
  ScanSummary filter(const std::vector<double>& ranges) const {
    ScanSummary summary;

    for (const double range : ranges) {
      // NaN 和 Inf 不能参与后续几何计算，必须先丢弃。
      if (!std::isfinite(range)) {
        continue;
      }

      if (range < min_range_ || range > max_range_) {
        continue;
      }

      summary.valid_ranges.push_back(range);
      ++summary.valid_count;

      if (range < summary.nearest_range) {
        summary.nearest_range = range;
      }
    }

    return summary;
  }

 private:
  double min_range_;
  double max_range_;
};

void print_summary(const ScanSummary& summary) {
  std::cout << "有效距离数量: " << summary.valid_count << '\n';

  if (summary.valid_count == 0U) {
    std::cout << "这一帧没有有效距离。\n";
    return;
  }

  std::cout << "最近有效距离: " << std::fixed << std::setprecision(2)
            << summary.nearest_range << " m\n";
  std::cout << "有效距离: ";
  for (const double range : summary.valid_ranges) {
    std::cout << range << " ";
  }
  std::cout << '\n';
}
}  // namespace predictive_nav::tutorial

int main() {
  // 这是一个模拟的 LaserScan.ranges。quiet_NaN 和 infinity 在真实激光中可能出现。
  const std::vector<double> raw_ranges{
      0.03,
      0.20,
      1.25,
      std::numeric_limits<double>::quiet_NaN(),
      3.70,
      std::numeric_limits<double>::infinity(),
      8.50,
  };

  const predictive_nav::tutorial::RangeFilter filter(0.50, 6.00);
  const predictive_nav::tutorial::ScanSummary summary = filter.filter(raw_ranges);

  std::cout << "原始距离数量: " << raw_ranges.size() << '\n';
  predictive_nav::tutorial::print_summary(summary);
  return 0;
}
