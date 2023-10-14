#ifndef TANKROBOT_H
#define TANKROBOT_H

#include "api.h"
#include "misc.hpp"
#include "odometry.hpp"
#include "drivetrain.hpp"
#include "rollerIntake.hpp"

class TankRobot
{
    public:
        TankRobot(TankDrivetrain d, RollerIntake in, Odometry* odom, TeamColor tc, pros::Vision* v, PIDConstants drive, PIDConstants turn);
        void goTo(Coordinate c, double angle, int timeout);
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
        RollerIntake ri;
        pros::Vision* vision;
        pros::Optical* optical;
        TeamColor color;
        PIDConstants drivePID;
        PIDConstants turnPID;
        PIDController PIDControl;
};

#endif