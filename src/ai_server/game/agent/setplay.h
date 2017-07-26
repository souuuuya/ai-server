#ifndef AI_SERVER_GAME_AGENT_SETPLAY_H
#define AI_SERVER_GAME_AGENT_SETPLAY_H

#include <memory>
#include <vector>
#include <Eigen/Core>
#include "base.h"
#include "ai_server/model/world.h"
#include "ai_server/game/action/kick_action.h"
#include "ai_server/game/action/receive.h"

namespace ai_server {
namespace game {
namespace agent {

class setplay : public base {
public:
  // finished:動作が終わった状態, setup:指定位置へ移動, pass:パスを蹴る, receive:パスを受け取る,
  // shoot:シュートする
  enum class state { finished, setup, pass, receive, shoot };
  enum class pos { near, mid, far };
  setplay(const model::world& world, bool is_yellow, unsigned int kicker_id,
          const std::vector<unsigned int>& receiver_id);

  std::vector<unsigned int> free_robots() const;

  std::vector<std::shared_ptr<action::base>> execute() override;

  bool finished();

private:
  pos pos_;
  state state_ = state::setup;
  unsigned int kicker_id_;
  unsigned int shooter_id_;
  const std::vector<unsigned int> receiver_ids_;
  Eigen::Vector2d prev_ball_vel_;
  std::shared_ptr<action::kick_action> kick_;
  std::shared_ptr<action::receive> receive_;
  Eigen::Vector2d passpos_;
  Eigen::Vector2d shoot_pos;
  std::vector<Eigen::Vector2d> positions_;
  bool neflag = false;
  bool receive_flag_ = false;
  int shooter_num_ = 0;
  int change_count_  = 0;
  double ballysign;

  std::vector<unsigned int> free_robots_;

  int mode_;

  int chose_location(std::vector<Eigen::Vector2d> targets,
                                model::world::robots_list enemy_robots, int dist = -1);
  double vectorangle(Eigen::Vector2d vec);
};
} // agent
} // game
} // ai_server
#endif
