#ifndef SPOOL_HPP
#define SPOOL_HPP

#include "pros/motors.hpp"

enum SpoolPosition
{
    RETRACTED,
    SWEEP,
    EXTENDED
};

class Spool
{
    public:
        const int STANDARD_MV = 12000;
        Spool(pros::MotorGroup& m, int extendedPosition);
        void moveTo(SpoolPosition pos);
        void spin(int voltage);
        SpoolPosition getPosition();
    private:
        pros::MotorGroup& motors;
        SpoolPosition position;
        double extendedPos;
};

#endif