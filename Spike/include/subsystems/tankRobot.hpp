#ifndef TANKROBOT_H
#define TANKROBOT_H

#include "api.h"
#include "misc.hpp"
#include "odometry.hpp"
#include "drivetrain.hpp"
#include "rollerIntake.hpp"
#include "indexer.hpp"
#include "visionsensor.hpp"
#include "conveyor.hpp"
#include "hood.hpp"

class TankRobot
{
    public:
        TankRobot(TankDrivetrain& d, RollerIntake& in, Indexer* i, Hood* h, Conveyor* conveyor, VisionSensor* vis, TeamColor tc);
        void goTo(Coordinate c, int timeout);
        void driveTo(Coordinate c, int timeout);
        void turnTo(double angle, int timeout);
        void autoAim(bool useVision);
        void autoShoot(bool useVision);
        void pollController(bool dualDriver);

    private:
        TankDrivetrain& drivetrain;
        pros::Controller driver;
        pros::Controller partner;
        RollerIntake& ri;
        Conveyor* conveyor;
        Indexer* indexer;
        Hood* hood;
        TeamColor color;
};

#endif