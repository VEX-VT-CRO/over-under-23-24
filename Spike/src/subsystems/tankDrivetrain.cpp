#include "subsystems/drivetrain.hpp"

#include <cmath>
#include <cstdarg>

TankDrivetrain::TankDrivetrain(pros::Motor* l, pros::Motor* r, int count)
{
    left = l;
    right = r;
    motorCount = count;
}

void TankDrivetrain::drive(int mV)
{
    for(int i = 0; i < motorCount; ++i)
    {
        left[i].move_voltage(mV);
        right[i].move_voltage(mV);
    }
}

void TankDrivetrain::turnLeft(int mV)
{
    for(int i = 0; i < motorCount; ++i)
    {
        left[i].move_voltage(-mV);
        right[i].move_voltage(mV);
    }
}

pros::Motor* TankDrivetrain::getLeft()
{
    return left;
}

pros::Motor* TankDrivetrain::getRight()
{
    return right;
}

void TankDrivetrain::tankControl(pros::Controller& c)
{
    int l = (c.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) * 120 / 127) * 100;
    int r = (c.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) * 120 / 127) * 100;
    for(int i = 0; i < motorCount; ++i)
    {
        left[i].move_voltage(l);
        right[i].move_voltage(r);
    }
}

void TankDrivetrain::arcadeControl(pros::Controller& c)
{
    //Power
    int l = (c.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) * 120 / 127) * 100;
    
    //Normalize direction [-127, 127] -> [0, 2], 1 being no input on the stick
    double r = (c.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) + 127) / 127.0;

    for(int i = 0; i < motorCount; ++i)
    {
        left[i].move_voltage(static_cast<int>(l * r));
        right[i].move_voltage(static_cast<int>(l * (1 - r)));
    }
}