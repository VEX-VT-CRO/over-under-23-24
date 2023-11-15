#ifndef TANKROBOT_H
#define TANKROBOT_H

#include "api.h"
#include "misc.hpp"
#include "odometry.hpp"
#include "drivetrain.hpp"
#include "rollerIntake.hpp"
#include "turret.hpp"
#include "catapult.hpp"
#include "indexer.hpp"
#include "visionsensor.hpp"

class TankRobot
{
    public:
        TankRobot(TankDrivetrain& d, RollerIntake& in, Indexer i, Turret* t, VisionSensor* vis, Odometry* odom, Catapult* catapult, TeamColor tc, PIDConstants drive, PIDConstants turn);
        void goTo(Coordinate c, int timeout);
        void driveTo(Coordinate c, int timeout);
        void turnTo(double angle, int timeout);
        void autoAim(bool useVision);
        void autoShoot(bool useVision);
        void pollController(bool dualDriver);

    private:
        TankDrivetrain drivetrain;
        Odometry* odometry;
        pros::Controller driver;
        pros::Controller partner;
        RollerIntake& ri;
        Turret* turret;
        Catapult* catapult;
        Indexer indexer;
        TeamColor color;
        PIDConstants drivePID;
        PIDConstants turnPID;
        PIDController PIDControl;
};

#endif