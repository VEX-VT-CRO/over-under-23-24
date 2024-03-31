#include "subsystems/drivetrain.hpp"

#include <cmath>
#include <cstdarg>

TankDrivetrain::TankDrivetrain(pros::Motor_Group& l, pros::Motor_Group& r) : left{l}, right{r}
{
}

void TankDrivetrain::drive(int mV)
{
    left.move_voltage(mV);
    right.move_voltage(mV);
}

void TankDrivetrain::turnLeft(int mV)
{
    left.move_voltage(-mV);
    right.move_voltage(mV);
}

pros::Motor_Group& TankDrivetrain::getLeft()
{
    return left;
}

pros::Motor_Group& TankDrivetrain::getRight()
{
    return right;
}

void TankDrivetrain::tankControl(pros::Controller& c)
{
    int l = (c.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) * 120 / 127) * 100;
    int r = (c.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) * 120 / 127) * 100;
    left.move_voltage(l);
    right.move_voltage(r);
}