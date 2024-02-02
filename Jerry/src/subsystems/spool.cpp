#include "subsystems/spool.hpp"


Spool::Spool(pros::MotorGroup& m, int extendedPosition) : motors{m}
{
    extendedPos = extendedPosition;
    position = SpoolPosition::RETRACTED;

    motors.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
}

void Spool::moveTo(SpoolPosition pos)
{
    position = (position == SpoolPosition::EXTENDED) ? SpoolPosition::RETRACTED : SpoolPosition::EXTENDED;
    switch(position)
    {
    case RETRACTED:
        motors.move_absolute(extendedPos, 200);
        break;
    case EXTENDED:
        motors.move_absolute(0, 200);
        break;
    default:
        break;
    }
}

SpoolPosition Spool::getPosition()
{
    return position;
}