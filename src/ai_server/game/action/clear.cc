
#include <cmath>
#include "clear.h"
#include "ai_server/util/math/angle.h"
#include "ai_server/util/math/to_vector.h"
#include "ai_server/util/math/distance.h"
#include "ai_server/util/math/geometry.h"
#include "ai_server/model/motion/stop.h"
#include "ai_server/model/motion/walk_forward.h"
#include "ai_server/model/motion/turn_left.h"
#include "ai_server/model/motion/turn_right.h"
#include "ai_server/game/detail/mcts.h"
#include "ai_server/model/field.h"

#include <iostream>
using namespace std;

namespace ai_server::game::action {

clear::clear(context& ctx, unsigned int id) : base(ctx, id) {}

bool clear::finished() const {

    return false;
}

model::command clear::execute() {
    model::command command{};

    const auto our_robots = model::our_robots(world(), team_color());

    if (!our_robots.count(id_)) return command;

    const auto robot = our_robots.at(id_);

    //自分とボールとゴールの位置を取得
    const auto robot_pos = util::math::position(robot);
    const auto ball_pos = util::math::position(world().ball());
    const Eigen::Vector2d ene_goal_pos(world(). field(). x_min(), 0.0);

    
    //２点の距離・角度、２つのベクトルの角度について
    const double kyori = util::math::distance(ball_pos, robot_pos);
    const double kakudo = util::math::direction(ball_pos, robot_pos);
    const double omega = util::math::direction_from(std::atan2(ball_pos.y() - robot_pos.y(), ball_pos.x() - robot_pos.x()), robot.theta());
    
    //距離の変数
    constexpr double a = 350;
    //ラストポント（ボールの裏）
    //Eigen::Vector2d target0_pos = ball_pos + util::math::direction(ball_pos, ene_goal_pos) * (ball_pos - ene_goal_pos).normalized();
     Eigen::Vector2d target0_pos = ball_pos + a * (ball_pos - ene_goal_pos).normalized();


    //p1,p2　leftP,rightP=
    Eigen::Vector2d p1, p2, leftP, rightP;
    std::tie(p1, p2) = util::math::calc_isosceles_vertexes(ene_goal_pos, ball_pos, a);
    std::tie(leftP, rightP) = util::math::contact_points(ball_pos, robot_pos, a);

    //p1・p2からロボットまでの距離
    const double P1_kyori = util::math::distance(p1, robot_pos);
    const double P2_kyori = util::math::distance(p2, robot_pos);

    std::cout << "MYロボット" << robot_pos << "\n";
    std::cout << "ボール" << ball_pos << "\n";
    std::cout << "相手ゴール" << ene_goal_pos << "\n";
    std::cout << "距離:" << kyori << "\n";
    std::cout << "角度:" << kakudo << "\n";
    std::cout << "角度差:" << omega << "\n";
    std::cout << "Lball:" << p1 << "\n";
    std::cout << "Rball:" << p2 << "\n";
    std::cout << "ラスト点" << target0_pos << "\n";

    
/*-----------------------------------動作-----------------------------------------------------*/


    
   //ボールの方向を向きつつボールの位置に移動
    command.set_position(ball_pos, util::math::direction(ball_pos, robot_pos));


    //p1,p2に進む
    if (P1_kyori < P2_kyori){
        command.set_position(p1, util::math::direction(ball_pos, p1));
    } else if (P1_kyori > P2_kyori){
        command.set_position(p2, util::math::direction(ball_pos, p2));
    }
    
        //ラストポントに移動    
        command.set_position(target0_pos, util::math::direction(ball_pos, target0_pos));
    

    //前進
    //command.set_motion(std::make_shared<model::motion::walk_forward>());
    

    constexpr double rot_th = 0.5;
    const double pai = 3.14;

    if (rot_th < omega) {
        if(omega <= 2*pai && omega >= pai){    
            command.set_motion(std::make_shared<model::motion::turn_right>());
        }else{
            command.set_motion(std::make_shared<model::motion::turn_left>());
        }

    } else if (omega < -rot_th) {
        command.set_motion(std::make_shared<model::motion::turn_right>());
    }
    return command;
}

}