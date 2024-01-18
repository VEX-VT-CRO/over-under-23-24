#include "subsystems/tankRobot.hpp"
#include <cmath>

TankRobot::TankRobot(TankDrivetrain& d, RollerIntake& in, Indexer* i, Turret* t, VisionSensor* vis, Catapult* catapult, TeamColor tc) : 
    drivetrain{d}, ri{in}, indexer{i}, turret{t}, color{tc}, driver{pros::Controller(pros::E_CONTROLLER_MASTER)}, 
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
    static bool toggle_pneumatics = false;
    static bool toggle_intake = false;
    drivetrain.tankControl(driver);

    if(!dualDriver)
    {
        if(driver.get_digital(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_L1))
        {
            ri.spin(6000);
        }
        else if(driver.get_digital(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_L2))
        {
            ri.spin(-6000);
        }
        else
        {
            ri.spin(0);
        }
    }

    //Toggle manual aim if driver presses A (once per new press)
    
    
    if(!dualDriver){
        if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))
            indexer->indexDisc(toggle_pneumatics);
    }
    else{
        if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))
            indexer->indexDisc(toggle_pneumatics);
        if(partner.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))
            indexer->indexDisc(false);
        if(partner.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2))
            indexer->indexDisc(true);    
    }

    /*if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
        catapult->shoot(3000);
        pros::delay(10);
        catapult->charge();
    }*/

    pros::lcd::clear();
}