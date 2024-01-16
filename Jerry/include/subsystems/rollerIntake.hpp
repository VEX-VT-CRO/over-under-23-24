#ifndef ROLLERINTAKE_HPP
#define ROLLERINTAKE_HPP

#include "pros/motors.hpp"

class RollerIntake
{
    public:
        const int STANDARD_MV = 12000;
        RollerIntake(pros::Motor& m1,  pros::Motor& m2);
        void spin(int mV);
        //void switchColor(bool useOptical);
    private:
        pros::Motor& motor1;
        pros::Motor& motor2;
};

#endif