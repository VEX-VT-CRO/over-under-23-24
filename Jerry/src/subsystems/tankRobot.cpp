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
}

void TankRobot::pollController(bool dualDriver)
{
    static bool manualAim = false;
    constexpr int TURRET_SPEED = 6000;
    drivetrain.tankControl(driver);

    if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))
    {
        ri.spin(-ri.STANDARD_MV);
    }
    else if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
    {
        ri.spin(ri.STANDARD_MV);
    }
    else
    {
        ri.spin(0);
    }

    if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2))
    {
        if (!catapult->shoot_ready){
            catapult->charge_state = true;
            pros::delay(20);
            turret->rotateback();
        }    
        else{
            catapult->shoot_state = true; 
            catapult->shoot_ready = false;
        }      
    }

    if(catapult->free_move)
        if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
            catapult->spin(-6000);
        else
            catapult->spin(0);

    if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
    {
        spool->moveTo(SpoolPosition::RETRACTED);
    }

    if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
        catapult->free_move = !catapult->free_move;
    }

    if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
            turret->updatePosition();
    }
    
    manualAim = (driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) ? !manualAim : manualAim;
    
    if(dualDriver){
        int turretDirection = partner.get_digital(pros::E_CONTROLLER_DIGITAL_L1) - driver.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        turret->turnVoltage(TURRET_SPEED * turretDirection);

    }
    if(manualAim)
    {
        int turretDirection = driver.get_digital(pros::E_CONTROLLER_DIGITAL_L1) - driver.get_digital(pros::E_CONTROLLER_DIGITAL_R1); 
        turret->turnVoltage(TURRET_SPEED * turretDirection);
    }

    pros::lcd::clear();
}