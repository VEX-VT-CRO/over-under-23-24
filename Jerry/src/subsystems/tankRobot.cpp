#include "subsystems/tankRobot.hpp"
#include <cmath>

TankRobot::TankRobot(TankDrivetrain d, RollerIntake& in, Turret& t, Odometry& odom, TeamColor tc, PIDConstants drive, PIDConstants turn) : 
    drivetrain{d}, ri{in}, turret{t}, odometry{odom}, color{tc}, driver{pros::Controller(pros::E_CONTROLLER_MASTER)}, 
    partner{pros::Controller(pros::E_CONTROLLER_PARTNER)}, PIDControl{PIDController(drivePID)}, drivePID{drive}, turnPID{turn}
{
    
}

void TankRobot::goTo(Coordinate c, double angle, int timeout)
{
    //Implement
    return;
}

void TankRobot::driveTo(Coordinate c, int timeout)
{
    PIDControl.goToTarget(drivetrain, c, odometry, timeout);
}

void TankRobot::turnTo(double angle, int timeout)
{
    PIDControl.goToAngle(drivetrain, angle, odometry, timeout);
}

void TankRobot::autoAim(bool useVision)
{
    Coordinate goal = {122.63, 122.63};
    Coordinate r = odometry.getPosition();
    //double angle = std::atan2(goal.y - r.y, goal.x - r.x);
}

void TankRobot::pollController(bool dualDriver)
{
    drivetrain.tankControl(driver);

    if(!dualDriver)
    {
        if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            ri.spin(ri.STANDARD_MV);
        }
        else if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            ri.spin(ri.STANDARD_MV);
        }
        else
        {
            ri.spin(0);
        }
    }

    odometry.update();
    Coordinate c = odometry.getPosition();
    pros::lcd::clear();
    pros::lcd::print(0, "X: %f", c.x);
    pros::lcd::print(1, "Y: %f", c.y);
    //pros::lcd::print(2, "A: %f", odometry->getAngle());
}