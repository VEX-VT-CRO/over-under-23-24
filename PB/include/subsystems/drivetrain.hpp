#ifndef DRIVETRAIN_HPP
#define DRIVETRAIN_HPP

#include <array>
#include "api.h"
#include "misc.hpp"

class Drivetrain
{
    public:
        virtual void drive(int mV);
        virtual void turnLeft(int mV);
};

class TankDrivetrain
{
    public:
        TankDrivetrain(pros::Motor_Group& l, pros::Motor_Group& r);
        
        void drive(int mV);
        void turnLeft(int mV);
        void tankControl(pros::Controller& c);
        //void driveForward(double inches, int timeout);
        //void turnTo(double angles, int timeout);
        pros::Motor_Group& getLeft();
        pros::Motor_Group& getRight();

    private:
        pros::Motor_Group& left;
        pros::Motor_Group& right;
};

#endif