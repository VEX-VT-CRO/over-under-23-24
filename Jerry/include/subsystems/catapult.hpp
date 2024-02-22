#ifndef CATAPULT_HPP
#define CATAPULT_HPP

#include "api.h"
#include "odometry.hpp"

class Catapult
{
    public:
        Catapult(pros::Motor* m, pros::Rotation& rotated);
        void shoot();
        void charge();
        void spin(int mV);
        void half();
        bool charge_state;
        bool shoot_state;
        bool shoot_ready;
        bool free_move;
        int triggered;
    private:
        pros::Motor* motor;
        pros::Rotation rotate;
        double angle;
};

#endif