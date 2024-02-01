#include "subsystems/tankRobot.hpp"
#include <cmath>

TankRobot::TankRobot(TankDrivetrain& d, RollerIntake& in, Indexer* i, Turret* t, VisionSensor* vis, Catapult* catapult, Spool* s, TeamColor tc) : 
    drivetrain{d}, ri{in}, indexer{i}, turret{t}, color{tc}, driver{pros::Controller(pros::E_CONTROLLER_MASTER)}, 
    partner{pros::Controller(pros::E_CONTROLLER_PARTNER)}
{
    this->catapult = catapult;
    this->spool = s;
}

void TankRobot::autoAim(bool useVision)
{
    Coordinate goal = {122.63, 122.63};
    //Coordinate r = odometry->getPosition();
    //double angle = std::atan2(goal.y - r.y, goal.x - r.x);
}

void TankRobot::pollController(bool dualDriver)
{
    static bool manualAim = false;
    static bool toggle_pneumatics = false;
    drivetrain.tankControl(driver);

    if(!dualDriver)
    {
        if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))
        {
            ri.spin(ri.STANDARD_MV);
        }
        else if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {
            ri.spin(-ri.STANDARD_MV);
        }
        else
        {
            ri.spin(0);
        }

        if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_X))
        {
            catapult->charge();
        }
        else if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
        {
            catapult->spin(-6000);
        }
        else
        {
            catapult->spin(0);
        }

        if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
        {
            SpoolPosition pos = (spool->getPosition() == SpoolPosition::RETRACTED) ? SpoolPosition::EXTENDED : SpoolPosition::RETRACTED;
            spool->moveTo(pos);
            pros::lcd::print(5, "pos: %d", pos);
        }
    }
    // if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
    //         turret->turnAngle(360);
    // }
    //Toggle manual aim if driver presses A (once per new press)

    //manualAim = (driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) ? !manualAim : manualAim;
    //toggle_pneumatics = (driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) ? !toggle_pneumatics : toggle_pneumatics;

    /*if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
        catapult->shoot(3000);
        pros::delay(10);
        catapult->charge();
    }*/
    
    /*if(manualAim)
    {
        constexpr int TURRET_SPEED = 9000;
        int turretDirection = driver.get_digital(pros::E_CONTROLLER_DIGITAL_L1) - driver.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        
        turret->turnVoltage(TURRET_SPEED * turretDirection);
    }*/

    pros::lcd::clear();
}