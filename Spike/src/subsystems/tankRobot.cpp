#include "subsystems/tankRobot.hpp"
#include <cmath>

TankRobot::TankRobot(TankDrivetrain& d, RollerIntake& in, Indexer i, Turret* t, VisionSensor* vis, Odometry* odom, Catapult* catapult, TeamColor tc, PIDConstants drive, PIDConstants turn) : 
    drivetrain{d}, ri{in}, indexer{i}, turret{t}, odometry{odom}, color{tc}, driver{pros::Controller(pros::E_CONTROLLER_MASTER)}, 
    partner{pros::Controller(pros::E_CONTROLLER_PARTNER)}, PIDControl{PIDController(drivePID)}, drivePID{drive}, turnPID{turn}
{
    
}

void TankRobot::goTo(Coordinate c, int timeout)
{
    constexpr double RAD2DEG = 180 / PI;
    Coordinate pos = odometry->getPosition();
    double angle = atan2(c.y - pos.y, c.x - pos.x) * RAD2DEG;
    PIDControl.goToAngle(drivetrain, angle, *odometry, timeout);
    PIDControl.goToTarget(drivetrain, c, *odometry, timeout);
}

void TankRobot::driveTo(Coordinate c, int timeout)
{
    PIDControl.goToTarget(drivetrain, c, *odometry, timeout);
}

void TankRobot::turnTo(double angle, int timeout)
{
    PIDControl.goToAngle(drivetrain, angle, *odometry, timeout);
}

void TankRobot::autoAim(bool useVision)
{
    Coordinate goal = {122.63, 122.63};
    Coordinate r = odometry->getPosition();
    //double angle = std::atan2(goal.y - r.y, goal.x - r.x);
}

void TankRobot::pollController(bool dualDriver)
{
    static bool manualAim = false;
    static bool toggle_pneumatics = false;
    drivetrain.arcadeControl(driver);

    /*if(!dualDriver)
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
    }*/

    //Toggle manual aim if driver presses A (once per new press)
    manualAim = (driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) ? !manualAim : manualAim;
    toggle_pneumatics = (driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) ? !toggle_pneumatics : toggle_pneumatics;
    
    if(!dualDriver){
        if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))
            indexer.indexDisc(toggle_pneumatics);
    }
    else{
        if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))
            indexer.indexDisc(toggle_pneumatics);
        if(partner.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))
            indexer.indexDisc(false);
        if(partner.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2))
            indexer.indexDisc(true);    
    }

    if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
        catapult->shoot(3000);
        pros::delay(10);
        catapult->charge();
    }
    
    if(manualAim)
    {
        constexpr int TURRET_SPEED = 9000;
        int turretDirection = driver.get_digital(pros::E_CONTROLLER_DIGITAL_L1) - driver.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        
        turret->turnVoltage(TURRET_SPEED * turretDirection);
    }

    pros::lcd::clear();
    odometry->update();
}