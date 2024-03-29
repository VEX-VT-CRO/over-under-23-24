#ifndef TANKROBOT_H
#define TANKROBOT_H

#include "api.h"
#include "misc.hpp"
#include "odometry.hpp"
#include "drivetrain.hpp"
#include "rollerIntake.hpp"
#include "catapult.hpp"
#include "indexer.hpp"
#include "spool.hpp"
#include "visionsensor.hpp"
#include "spool.hpp"

class TankRobot
{
    public:
        TankRobot(TankDrivetrain& d, RollerIntake& in, Indexer* i, VisionSensor* vis, Catapult* catapult, Spool* s, TeamColor tc);
        void goTo(Coordinate c, int timeout);
        void driveTo(Coordinate c, int timeout);
        void turnTo(double angle, int timeout);
        void autoAim(bool useVision);
        void autoShoot(bool useVision);
        void pollController(bool dualDriver);
        void start();

    private:
        TankDrivetrain drivetrain;
        pros::Controller driver;
        pros::Controller partner;
        RollerIntake& ri;
        Catapult* catapult;
        Indexer* indexer;
        Spool* spool;
        TeamColor color;
};

#endif