#ifndef CLIMB_HPP
#define CLIMB_HPP

#include "pros/motors.hpp"

class Climb
{
    public:
        Climb(pros::Motor_Group& climbMotors);
        void moveClimb(int mV);
        void deployClimb_PB();
        void deployClimb_J();
    private:
        pros::Motor_Group& climbMotors;
};

#endif