#include <cmath>
#include "clear.h"
#include "ai_server/util/math/angle.h"
#include "ai_server/util/math/to_vector.h"
#include "ai_server/util/math/distance.h"
#include "ai_server/util/math/geometry.h"
#include "ai_server/model/motion/walk_forward.h"
#include "ai_server/model/motion/turn_left.h"
#include "ai_server/model/motion/turn_right.h"

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

    const auto robot_pos = util::math::position(robot);
    const auto ball_pos = util::math::position(world().ball());

    const double kyori = util::math::distance(ball_pos, robot_pos);
    const double kakudo = util::math::direction(ball_pos, robot_pos);
    const double omega = util::math::direction_from(std::atan2(ball_pos.y() - robot_pos.y(), ball_pos.x() - robot_pos.x()), robot.theta());
    
    std::cout << "距離:" << kyori << "\n";
    std::cout << "角度:" << kakudo << "\n";
    std::cout << "角度差:" << omega << "\n";

    command.set_position(ball_pos, util::math::direction(ball_pos, robot_pos));


    Eigen::Vector2d p1, p2, leftP, rightP;
    std::tie(p1, p2) = util::math::calc_isosceles_vertexes(robot_pos, ball_pos, kyori);
    std::tie(leftP, rightP) = util::math::calc_isosceles_vertexes(robot_pos, ball_pos, kyori);

    std::cout << "ベクトル左" << p1 << "\n";
    std::cout << "ベクトル右" << p2 << "\n";

    
    command.set_motion(std::make_shared<model::motion::walk_forward>());
    
    constexpr double rot_th = 0.1;
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