#include "subsystems/tankRobot.hpp"
#include <cmath>

TankRobot::TankRobot(TankDrivetrain& d, RollerIntake& in, Indexer* i, Conveyor* conveyor, VisionSensor* vis, TeamColor tc) : 
    drivetrain{d}, ri{in}, indexer{i}, color{tc}, driver{pros::Controller(pros::E_CONTROLLER_MASTER)}, 
    partner{pros::Controller(pros::E_CONTROLLER_PARTNER)}
{
    this->conveyor = conveyor;
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

        if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            conveyor->set(12000);
        }
        else if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            conveyor->set(-12000);
        }
        else
        {
            conveyor->set(0);
        }
    }

    if(!dualDriver){
        if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
            indexer->indexDisc();
    }

    pros::lcd::clear();
}