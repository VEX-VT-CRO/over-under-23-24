#include "subsystems/catapult.hpp"
#include <cmath>
#include "lemlib/chassis/odom.hpp"

Catapult::Catapult(pros::Motor* m, pros::ADIDigitalIn* catapult_charged, pros::Distance* distance_sensor){
    goal_position = {0,0,0};
    motor = m;
    charged = catapult_charged;
    triball_in = distance_sensor;
    charged = catapult_charged;
    current_position = odometry->getPosition();
    distance = 0;
    mV = 0;
    triball_distance = 50;
}

//use lemlib::getPos() to grab the position of the robot

// Function for finding the angle/velocity for launching the projectile knowing the required distance to travel
// void Catapult::findGoalSpeed(Coordinate goal_position, Coordinate current_position){
//     double dx = (current_position.x-goal_position.x);
//     double dy = (current_position.y-goal_position.y);
//     distance = std::sqrt(dx*dx+dy*dy);
//     mV = mV_const*std::sqrt(distance);
// }

void Catapult::shoot(int mV){
    if(triball_in->get()<triball_distance)
        motor->move_voltage(mV);
}

void Catapult::charge(){
    while (!charged->get_value()){
        motor->move_voltage(6000);
    }
}
