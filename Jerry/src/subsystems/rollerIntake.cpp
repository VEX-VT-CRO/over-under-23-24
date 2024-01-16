#include "subsystems/rollerIntake.hpp"

RollerIntake::RollerIntake(pros::Motor& m1, pros::Motor& m2) : motor1{m1}, motor2{m2}
{

}

void RollerIntake::spin(int mV)
{
    motor1.move_voltage(mV);
    motor2.move_voltage(-mV);
}