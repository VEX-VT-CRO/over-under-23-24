#include "subsystems/tankRobot.hpp"
#include <cmath>

TankRobot::TankRobot(TankDrivetrain& d, RollerIntake& in, Indexer* i, Hood* hood, Conveyor* conveyor, VisionSensor* vis, TeamColor tc) : 
    drivetrain{d}, ri{in}, indexer{i}, color{tc}, driver{pros::Controller(pros::E_CONTROLLER_MASTER)}, 
    partner{pros::Controller(pros::E_CONTROLLER_PARTNER)}
{
    this->conveyor = conveyor;
    this->hood = hood;
}

void TankRobot::autoAim(bool useVision)
{
    Coordinate goal = {122.63, 122.63};
}

void TankRobot::pollController(bool dualDriver)
{
    static bool manualAim = false;
    static bool toggle_intake = false;
    drivetrain.tankControl(driver);
    if(driver.get_digital(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_R1))
    {
        ri.spin(12000);
    }
    else if(driver.get_digital(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_R2))
    {
        ri.spin(-12000);
    }
    else
    {
        ri.spin(0);
    }
    if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
    {
        conveyor->set(12000);
    }
    else if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
    {
        conveyor->set(-12000);
    }
    else
    {
        conveyor->set(0);
    }
    if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
        indexer->indexDisc();
    if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
        hood->indexDisc();
    pros::lcd::clear();
}