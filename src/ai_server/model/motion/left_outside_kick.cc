#include "left_outside_kick.h"

namespace ai_server::model::motion {

left_outside_kick::left_outside_kick() : base(28) {}

std::tuple<double, double, double> left_outside_kick::execute() {
  return std::make_tuple<double, double, double>(-100.0, 0.0, 0.0);
}

} // namespace ai_server::model::motion