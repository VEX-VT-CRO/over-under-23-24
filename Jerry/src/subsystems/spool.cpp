#include "subsystems/spool.hpp"


Spool::Spool(pros::MotorGroup& m, int extendedPosition) : motors{m}
{
    extendedPos = extendedPosition;
    position = SpoolPosition::RETRACTED;

    motors.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
}

void Spool::moveTo(SpoolPosition pos)
{
    switch(pos)
    {
    case RETRACTED:
        //motors.move_absolute(extendedPos, 200);
        motors.move_voltage(12000);
        position = SpoolPosition::EXTENDED;
        break;
    case EXTENDED:
        //motors.move_absolute(0, 200);
        motors.move_voltage(-12000);
        position = SpoolPosition::RETRACTED;
        break;
    default:
        break;
    }
}

SpoolPosition Spool::getPosition()
{
    return position;
}