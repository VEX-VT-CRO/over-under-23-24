#include "subsystems/conveyor.hpp"

Conveyor::Conveyor(pros::MotorGroup& m) : motors{m}
{
    motors.set_brake_modes(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
}

void Conveyor::set(int mV)
{
    motors.move_voltage(mV);
}