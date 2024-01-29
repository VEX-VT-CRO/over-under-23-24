#include "subsystems/tankRobot.hpp"
#include <cmath>

TankRobot::TankRobot(TankDrivetrain& d, RollerIntake& in, Indexer* i, VisionSensor* vis, Catapult* catapult, TeamColor tc) : 
    drivetrain{d}, ri{in}, indexer{i}, color{tc}, driver{pros::Controller(pros::E_CONTROLLER_MASTER)}, 
    partner{pros::Controller(pros::E_CONTROLLER_PARTNER)}
{
    this->catapult = catapult;
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
    drivetrain.arcadeControl(driver);

    if(!dualDriver)
    {
        if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
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

        if (driver.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            if (!catapult->charge_state) {
                catapult->charge_state = true;
            }
        }

        if (driver.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            catapult->spin(-6000);
        } else {
            catapult->spin(0);
        }
        
        if (driver.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            catapult->charge_state = false;
        }

        if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
            indexer->indexDisc();
        }
    }

    //Toggle manual aim if driver presses A (once per new press)
    manualAim = (driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) ? !manualAim : manualAim;
    
    pros::lcd::clear();
}