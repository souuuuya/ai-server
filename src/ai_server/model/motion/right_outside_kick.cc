#include "right_outside_kick.h"

namespace ai_server::model::motion {

right_outside_kick::right_outside_kick() : base(19) {}

std::tuple<double, double, double> right_outside_kick::execute() {
  return std::make_tuple<double, double, double>(-100.0, 0.0, 0.0);
}

} // namespace ai_server::model::motion