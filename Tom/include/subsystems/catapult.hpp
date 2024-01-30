#ifndef CATAPULT_HPP
#define CATAPULT_HPP

#include "api.h"
#include "odometry.hpp"

class Catapult
{
    public:
        Catapult(pros::Motor* m, pros::ADIDigitalIn* catapult_charged, pros::Distance* distance_sensor);
        void charge();
        void findGoalSpeed(Coordinate goal_position, Coordinate current_position);
        void spin(int mV);
        void checkIfCharged();
        bool charge_state;
        bool shoot_state;
        pros::ADIDigitalIn* charged;
    private:
        pros::Motor* motor;
        pros::Distance* triball_in;
        double distance;
        Odometry* odometry;
        TeamColor color;
        Coordinate goal_position;
        Coordinate current_position;
        int triball_distance;
        double mV;
        //number need to be modified
        const double mV_const = 13.2;
};

#endif