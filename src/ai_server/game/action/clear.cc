
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
    const Eigen::Vector2d our_goal_pos(world(). field(). x_min(), 0.0);

    //ロボットの座標
    const auto rob_x = robot_pos.x();
    const auto rob_y = robot_pos.y();
    
    //２点の距離・角度、２つのベクトルの角度について
    const double kyori = util::math::distance(ball_pos, robot_pos);
    const double kakudo = util::math::direction(ball_pos, robot_pos);
    const double omega = util::math::direction_from(std::atan2(ball_pos.y() - robot_pos.y(), ball_pos.x() - robot_pos.x()), robot.theta());    
    const double Gkakudo = util::math::direction(ene_goal_pos, robot_pos);
    //const double kakudo_RG = util::math::direction(our_goal_pos, robot_pos);
    
    //距離の変数
    constexpr double a = 350;
    //ラストポント（ボールの裏）
    //Eigen::Vector2d target0_pos = ball_pos + util::math::direction(ball_pos, ene_goal_pos) * (ball_pos - ene_goal_pos).normalized();
     Eigen::Vector2d target0_pos = ball_pos + a * (ball_pos - ene_goal_pos).normalized();

    //
    Eigen::Vector2d target_hani = robot_pos - target0_pos;

    //p1,p2　leftP,rightP=
    Eigen::Vector2d p1, p2, leftP, rightP;
    std::tie(p1, p2) = util::math::calc_isosceles_vertexes(ene_goal_pos, ball_pos, a);
    std::tie(leftP, rightP) = util::math::contact_points(ball_pos, robot_pos, a);

    //p1・p2からロボットまでの距離
    const double P1_kyori = util::math::distance(p1, robot_pos);
    const double P2_kyori = util::math::distance(p2, robot_pos);

    //p1・p2の角度差
    const double omega_p1 = util::math::direction_from(std::atan2(p1.y() - robot_pos.y(), p1.x() - robot_pos.x()), robot.theta());    
    const double omega_p2 = util::math::direction_from(std::atan2(p2.y() - robot_pos.y(), p2.x() - robot_pos.x()), robot.theta());    
    const auto   omega_target0 = util::math::direction_from(std::atan2(target0_pos.y() - robot_pos.y(), target0_pos.x() - robot_pos.x()), robot.theta());

    std::cout << "------------------------" << "\n";

    std::cout << "MYロボットX" << rob_x << "\n";
    std::cout << "MYロボットy" << rob_y << "\n";
    std::cout << "ボールX" << ball_pos.x() << "\n";
    std::cout << "ボールY" << ball_pos.y() << "\n";
    //std::cout << "相手ゴール" << ene_goal_pos << "\n";
    std::cout << "RB距離:" << kyori << "\n";
    //std::cout << "RB角度:" << kakudo << "\n";
    //std::cout << "RB角度差:" << omega << "\n";
    //std::cout << "RGe角度" << Gkakudo << "\n";
    std::cout << "p2角度差:" << omega_p2 << "\n";
    std::cout << "ｐ１" << p1 << "\n";
    std::cout << "ｐ２" << p2 << "\n";
    //std::cout << "ラスト点X" << target0_pos.x() << "\n";
    //std::cout << "ラスト点Y" << target0_pos.y() << "\n";
    std::cout << "p1距離" << P1_kyori << "\n";
    std::cout << "p2距離" << P2_kyori << "\n";

    std::cout << "------------------------" << "\n";

/*-----------------------------------動作-----------------------------------------------------*/

    constexpr double rot_th = 0.5;
    constexpr double rot_th_p = 0.15;
    const double pai = 3.14;
    
   //ボールの方向を向きつつボールの位置に移動
    //command.set_position(ball_pos, util::math::direction(ball_pos, robot_pos));

    //前進
    command.set_motion(std::make_shared<model::motion::walk_forward>());

    //自分が自分ゴール側に向いている時と自分が相手ゴール側にいるとき
    if (kyori <= 650){

        std::cout << "条件その１入" << "\n";

        //Ｐ１（Ｐ２）の方向に向きながら前進
        if(P1_kyori < P2_kyori){

            std::cout << "p1に移動" << "\n";
           
            command.set_position(p1, util::math::direction(p1, robot_pos));

            //ｐ１の角度調整
            if (rot_th_p < omega_p1) {
                if(omega_p1 <= 2*pai && omega_p1 >= pai){    
                    command.set_motion(std::make_shared<model::motion::turn_right>());
                }else{
                    command.set_motion(std::make_shared<model::motion::turn_left>());
                }

            } else if (omega_p1 < -rot_th_p) {
                command.set_motion(std::make_shared<model::motion::turn_left>());
            }


        } else if(P1_kyori >= P2_kyori){

            std::cout << "p２に移動" << "\n";
           
            command.set_position(p2, util::math::direction(p2, robot_pos));

            
            //ｐ２の角度調整
            if (rot_th_p <= omega_p2) {
                if(omega_p2 <= 2*pai && omega_p2 >= pai){    
                    command.set_motion(std::make_shared<model::motion::turn_right>());
                }else{
                    command.set_motion(std::make_shared<model::motion::turn_left>());
                }

            } else if (omega_p2 <= -rot_th_p) {
                command.set_motion(std::make_shared<model::motion::turn_left>());
            }


                
                
        }

        //ラストポントへ移動
        if((p1.y() < rob_y+30 && p1.y() > rob_y-30) || (p2.y() < rob_y+30 && p2.y() > rob_y-30)){

            std::cout << "p１かp２に移動完了" << "\n";
            command.set_position(target0_pos, util::math::direction(target0_pos, robot_pos));
                    
             //ラストポントの角度調整
            /*if (rot_th_p < omega_target0) {
                if(omega_target0 <= 2*pai && omega_target0 >= pai){    
                    command.set_motion(std::make_shared<model::motion::turn_right>());
                }else{
                    command.set_motion(std::make_shared<model::motion::turn_left>());
                }

            } else if (omega_target0 < -rot_th_p) {
                command.set_motion(std::make_shared<model::motion::turn_left>());
            }*/
        }    

    }

    

    if (rot_th < omega) {
        if(omega <= 2*pai && omega >= pai){    
            command.set_motion(std::make_shared<model::motion::turn_right>());
        }else{
            command.set_motion(std::make_shared<model::motion::turn_left>());
        }

    } else if (omega < -rot_th) {
        command.set_motion(std::make_shared<model::motion::turn_left>());

    }
    return command;
}

}