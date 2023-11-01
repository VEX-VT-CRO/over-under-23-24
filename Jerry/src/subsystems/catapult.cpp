#include "subsystems/catapult.hpp"
#include <cmath>

Catapult::Catapult(pros::Motor* m, Odometry* odom){
    goal_position = {0,0,0};
    motor = m;
    current_position = odom->getPosition();
    distance = 0;
    mV = 0;
}

void Catapult::findGoalSpeed(Coordinate goal_position, Coordinate current_position){
    double dx = (current_position.x-goal_position.x);
    double dy = (current_position.y-goal_position.y);
    distance = std::sqrt(dx*dx+dy*dy);
    mV = mV_const*distance;
}

void Catapult::setSpeed(int mV){
    motor->move_voltage(mV);
}

