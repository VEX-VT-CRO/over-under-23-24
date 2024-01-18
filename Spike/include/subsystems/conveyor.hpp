#ifndef CONVEYOR_HPP
#define CONVEYOR_HPP

#include "pros/motors.hpp"

class Conveyor
{
    public:
        const int STANDARD_MV = 12000;
        Conveyor(pros::MotorGroup& m);
        void set(int mV);

    private:
        pros::MotorGroup& motors;
};

#endif