#include "subsystems/climb.hpp"

Climb::Climb(pros::Motor_Group& motors) : climbMotors{motors}
{
    climbMotors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
}

void Climb::moveClimb(int mV)
{
    climbMotors.move_voltage(mV);
}