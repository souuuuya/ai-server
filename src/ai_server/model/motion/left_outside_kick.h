#ifndef AI_SERVER_MODEL_MOTION_LEFT_OUTSIDE_KICK_H
#define AI_SERVER_MODEL_MOTION_LEFT_OUTSIDE_KICK_H

#include "base.h"

namespace ai_server::model::motion {

class left_outside_kick : public base {
public:
  left_outside_kick();

  std::tuple<double, double, double> execute() override;
};

} // namespace ai_server::model::motion

#endif