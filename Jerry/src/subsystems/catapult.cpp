#include "subsystems/catapult.hpp"
#include <cmath>

Catapult::Catapult(pros::Motor* m, Odometry* odom, pros::ADIDigitalIn* catapult_charged){
    goal_position = {0,0,0};
    motor = m;
    charged = catapult_charged;
    odometry = odom;
    charged = catapult_charged;
    current_position = odometry->getPosition();
    distance = 0;
    mV = 0;
}

// Function for finding the angle/velocity for launching the projectile knowing the required distance to travel
// void Catapult::findGoalSpeed(Coordinate goal_position, Coordinate current_position){
//     double dx = (current_position.x-goal_position.x);
//     double dy = (current_position.y-goal_position.y);
//     distance = std::sqrt(dx*dx+dy*dy);
//     mV = mV_const*std::sqrt(distance);
// }

void Catapult::shoot(int mV){
    motor->move_voltage(mV);
}

void Catapult::charge(){
    while (!charged->get_value()){
        motor->move_voltage(6000);
    }
}
