#include <boost/math/constants/constants.hpp>
#include <cmath>
#include "ai_server/util/math/angle.h"
#include "ai_server/util/math/to_vector.h"

#include "ai_server/model/motion/right_kick.h"
#include "ai_server/model/motion/right_outside_kick.h"
#include "ai_server/model/motion/left_kick.h"
#include "ai_server/model/motion/left_outside_kick.h"
#include "ai_server/model/motion/turn_left.h"
#include "ai_server/model/motion/turn_right.h"

#include "kick.h"

namespace ai_server {
namespace game {
namespace action {
kick::kick(context& ctx, unsigned int id)
    : base(ctx, id),
      mode_(mode::goal),
      state_(running_state::move),
      dribble_(0),
      margin_(0.05),
      finishflag_(false),
      stop_ball_flag_(false) {}

void kick::kick_to(double x, double y) {          //目標地点(ゴールの中心)
  target_ = Eigen::Vector2d{x, y};
}

void kick::set_kick_type(const model::command::kick_flag_t& kick_type) {          //キックの種類を決める(キックの種類はcommand.h の20行~22行)
  kick_type_ = kick_type;
}

// 蹴れる位置に移動するときに蹴る目標位置を見ているかボールを見ているか指定する関数
void kick::set_mode(mode mod) {
  mode_ = mod;
}

void kick::set_dribble(int dribble) {         //ドリブル
  dribble_ = dribble;
}

void kick::set_angle_margin(double margin) {          //の間、差
  margin_ = margin;
}

kick::running_state kick::state() const {         
  return state_;
}

void kick::set_stop_ball(bool stop_ball_flag) {         //ボールが止まるフラグ
  stop_ball_flag_ = stop_ball_flag;
}

model::command kick::execute() {
  using boost::math::constants::pi;         // ==180°
  using boost::math::constants::two_pi;     // ==360°

  const auto our_robots           = model::our_robots(world(), team_color());
  const auto enemy_robots         = model::enemy_robots(world(), team_color());
  const auto& robot_me            = our_robots.at(id_);
  const auto& robot_keeper        = enemy_robots.at(id_); 
  const Eigen::Vector2d robot_pos = util::math::position(robot_me);
  const Eigen::Vector2d robot_ene = util::math::position(robot_keeper);
  const Eigen::Vector2d ball_pos  = util::math::position(world().ball());
  const Eigen::Vector2d ball_vel  = util::math::velocity(world().ball());
  //ボールから目標の角度
  const double ball_target = std::atan2(target_.y() - ball_pos.y(), target_.x() - ball_pos.x());
  //ボールからロボットの角度
  const double ball_robot =
      std::atan2(robot_pos.y() - ball_pos.y(), robot_pos.x() - ball_pos.x());
  //ロボットからボールの角度
  const double robot_ball = ball_robot + pi<double>();　　//pi<double> == 180°
  //ロボットからキーパーロボットの角度
  const double robot_erobot = std::atan2(robot_pos.y() - robot_ene.y(), robot_pos.x() - robot_ene.x());
  //
  const auto keeper_theta = std::atan2(robot_ene.y(), robot_ene.x());
  //ボールとロボットの間の距離
  const double dist = 350;
  //送りたいロボットを指定
  model::command command{};

  // executeが呼ばれる間にボールがこれだけ移動したら蹴ったと判定する長さ(mm)
  const double kick_decision = 550;
  const bool kick_flag_tf    = std::get<0>(kick_type_) != model::command::kick_type_t::none;

  if ((ball_vel.norm() > kick_decision) || finishflag_) {
    // executeが呼ばれる間の時間でボールが一定以上移動をしていたら蹴ったと判定
    command.set_velocity({0, 0, 0});
    finishflag_ = true;
    return command;
  }

  switch (state_) {
    case running_state::move: { //ボールの近くまで寄る とき...

      if ((ball_pos - robot_pos).norm() < dist) {           //ロボットからボールまでの距離が dist より小さい時...
        state_ = running_state::round;
      }                                                                             //wrap_to_pi( 角度 − 基準の角度 )
      if (std::abs(util::math::wrap_to_pi(ball_target - robot_ball)) < margin_) {       //もし、{（ボールから目標の角度）-（ボールからロボットの角度）}　＜　0.05(2.86°)
        state_ = running_state::kick;
      }
      //ボールのそばによる処理
      command.set_position({std::cos(ball_target + pi<double>()) * dist + ball_pos.x(),
                            std::sin(ball_target + pi<double>()) * dist + ball_pos.y(),
                            ball_target});
    } break;

    case running_state::round: { //回り込 のとき...

      if ((ball_pos - robot_pos).norm() > 1.5 * dist) {         //ロボットからボールまでの距離が dist*1.5 より大きい時...
        state_ = running_state::move;
      }
      if (std::abs(util::math::wrap_to_pi(ball_target - robot_ball)) < margin_) {
        state_ = running_state::kick;
      }
      //回りこみ処理
      const double velo =
          (util::math::wrap_to_pi(ball_target + pi<double>() - ball_robot) / pi<double>()) *
          1500;
      const double si = -std::sin(ball_robot) * velo;
      const double co = std::cos(ball_robot) * velo;
      command.set_velocity(
          {si, co, 4.0 * util::math::wrap_to_pi(ball_target - robot_me.theta())});
    } break;

    case running_state::kick: { //キックの状態
      if (std::abs(util::math::wrap_to_pi(ball_target - robot_ball)) > 1.5 * margin_) {
        state_ = running_state::round;
      }
      if ((ball_pos - robot_pos).norm() > 1.5 * dist) {
        state_ = running_state::move;
      }
      //キック処理
      command.set_kick_flag(kick_type_);
      command.set_dribble(dribble_);
      const double velo =
          (util::math::wrap_to_pi(ball_target + pi<double>() - ball_robot) / pi<double>()) *
          1500;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      //キーパーが右                                                                                  //ロボットとボールの位置を見ているので、相手のきーぱーの位置を取得、それとぼーるの角度を確認してif（switch）で制御
      if(robot_ene.y() < 0){                                                                      //参考：キーパーロボットの開脚で守れる範囲は、ロボットを中心に30cm、シュートの入る角度（rad）は左右に0.3ずつ
        //左キック
        command.set_motion(std::make_shared<model::motion::left_kick>());

        //キーパーが極端に右
        if(pi<double>() / 7.2 < std::abs(std::atan2(robot_pos.y() - robot_ene.y(), robot_pos.x() - robot_ene.x()) - keeper_theta) < pi<double>() / 4.0){
          command.set_position(robot_ene);
          command.set_motion(std::make_shared<model::motion::left_outside_kick>());
        }


      //キーパーが左
      }else if(robot_ene.y() >0){
        //右キック
        command.set_motion(std::make_shared<model::motion::right_kick>());

        //キーパーが極端に左
        if(pi<double>() / -7.2 < std::abs(std::atan2(robot_pos.y() - robot_ene.y(), robot_pos.x() - robot_ene.x()) - keeper_theta) < pi<double>() / -4.0){
          command.set_position(robot_ene);
          command.set_motion(std::make_shared<model::motion::right_outside_kick>());
        }
      }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      const double si = -std::sin(ball_robot) * velo;
      const double co = std::cos(ball_robot) * velo;
      const double move_vel =
          !kick_flag_tf ? 0 : 3.0 * ((robot_pos - ball_pos).norm() - 75.0); // 90はドリブラー分               意味：○ =！「kick_flag_tf」が ture なら0、 false ならロボットからボールまでの距離 − 75
      const Eigen::Vector2d vel = move_vel * (ball_pos - robot_pos).normalized();                             
      command.set_velocity({vel.x() + si, vel.y() + co,                                                      
                            4.0 * util::math::wrap_to_pi(ball_target - robot_me.theta())});
    } break;
  }

  return command;

}
bool kick::finished() const {
  return finishflag_;
}
} // namespace action
} // namespace game
} // namespace ai_server
