#ifndef ROLLERINTAKE_HPP
#define ROLLERINTAKE_HPP

#include "pros/motors.hpp"

class RollerIntake
{
    public:
        const int STANDARD_MV = 12000;
        RollerIntake(pros::Motor& m);
        void spin(int mV);
        //void switchColor(bool useOptical);
    private:
        pros::Motor& motor;
};

#endif