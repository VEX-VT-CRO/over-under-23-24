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
        TankDrivetrain(pros::Motor* l, pros::Motor* r, int count);
        
        void drive(int mV);
        void turnLeft(int mV);
        void tankControl(pros::Controller& c);
        void arcadeControl(pros::Controller& c);
        //void driveForward(double inches, int timeout);
        //void turnTo(double angles, int timeout);
        pros::Motor* getLeft();
        pros::Motor* getRight();

    private:
        pros::Motor* left;
        pros::Motor* right;
        int motorCount;
};

#endif