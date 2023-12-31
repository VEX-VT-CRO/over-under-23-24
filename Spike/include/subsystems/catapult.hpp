#ifndef CATAPULT_HPP
#define CATAPULT_HPP

#include "api.h"
#include "odometry.hpp"

class Catapult
{
    public:
        Catapult(pros::Motor* m, Odometry* odom, pros::ADIDigitalIn* catapult_charged);
        void shoot(int mV);
        void charge();
        void findGoalSpeed(Coordinate goal_position, Coordinate current_position);
    private:
        pros::Motor* motor;
        pros::ADIDigitalIn* charged;
        double distance;
        Odometry* odometry;
        TeamColor color;
        Coordinate goal_position;
        Coordinate current_position;
        double mV;
        //number need to be modified
        const double mV_const = 13.2;
};

#endif