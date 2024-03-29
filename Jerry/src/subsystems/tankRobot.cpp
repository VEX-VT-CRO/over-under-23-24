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

/*void TankRobot::pollController(bool dualDriver)
{
    static bool manualAim = false;
    constexpr int TURRET_SPEED = 6000;
    static bool rapidshootmode = false;
    drivetrain.tankControl(driver);

    if(!rapidshootmode){
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

        if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1) && !manualAim)
        {
            SpoolPosition pos = (spool->getPosition() != SpoolPosition::EXTENDED) ? SpoolPosition::EXTENDED : SpoolPosition::RETRACTED;
            spool->moveTo(pos);
        }
        else if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
        {
            SpoolPosition pos = (spool->getPosition() != SpoolPosition::SWEEP) ? SpoolPosition::SWEEP : SpoolPosition::RETRACTED;
            spool->moveTo(pos);
        }

        if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
            catapult->free_move = !catapult->free_move;
        }

        if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
                turret->updatePosition();
        }
        
        if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
            turret->reset_angles();
        }
        manualAim = (driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) ? !manualAim : manualAim;
        
        if(dualDriver){
            int turretDirection = partner.get_digital(pros::E_CONTROLLER_DIGITAL_L1) - partner.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
            turret->turnVoltage(TURRET_SPEED * turretDirection);
            if(partner.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
                turret->reset_angles();
            }
            if(catapult->free_move)
                if(partner.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
                    catapult->spin(-6000);
                else
                    catapult->spin(0);
            if(partner.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
                catapult->free_move = !catapult->free_move;
        }        
        }
        if(manualAim)
        {
            int turretDirection = driver.get_digital(pros::E_CONTROLLER_DIGITAL_L1) - driver.get_digital(pros::E_CONTROLLER_DIGITAL_R1); 
            turret->turnVoltage(TURRET_SPEED * turretDirection);
        }
    }
    else{
        if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
            int extendTime = 750;
            catapult->charge_state = true;
            pros::delay(20);
            turret->rotateback(); 
            pros::delay(100);
            spool->moveTo(EXTENDED);
		    pros::delay(extendTime);
		    spool->moveTo(RETRACTED);
		    pros::delay(extendTime);
            turret->updatePosition();
            pros::delay(100);
            catapult->shoot_state = true; 
            catapult->shoot_ready = false;    
        }
    }    
    rapidshootmode = (driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) ? !rapidshootmode : rapidshootmode;
    pros::lcd::clear();
}*/

void TankRobot::pollController(bool dualDriver)
{
    static bool manualAim = false;
    static bool rapidshootmode = false;
    constexpr int TURRET_SPEED = 6000;
    drivetrain.tankControl(driver);


    if(!rapidshootmode){
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

        if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1) && !manualAim)
        {
            SpoolPosition pos = (spool->getPosition() != SpoolPosition::EXTENDED) ? SpoolPosition::EXTENDED : SpoolPosition::RETRACTED;
            spool->moveTo(pos);
        }
        else if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
        {
            SpoolPosition pos = (spool->getPosition() != SpoolPosition::SWEEP) ? SpoolPosition::SWEEP : SpoolPosition::RETRACTED;
            spool->moveTo(pos);
        }

        if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
            catapult->free_move = !catapult->free_move;
        }

        if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
                turret->updatePosition();
        }
        
        if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
            turret->reset_angles();
        }
        manualAim = (driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) ? !manualAim : manualAim;
        
        if(dualDriver && manualAim){
            int turretDirection = partner.get_digital(pros::E_CONTROLLER_DIGITAL_L1) - partner.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
            turret->turnVoltage(TURRET_SPEED * turretDirection);
            if(partner.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
                turret->reset_angles();
            }
            if(catapult->free_move)
                if(partner.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
                    catapult->spin(-6000);
                else
                    catapult->spin(0);
            if(partner.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
                catapult->free_move = !catapult->free_move;
        }        
        }
        if(manualAim)
        {
            //int turretDirection = driver.get_digital(pros::E_CONTROLLER_DIGITAL_L1) - driver.get_digital(pros::E_CONTROLLER_DIGITAL_R1); 
            //turret->turnVoltage(TURRET_SPEED * turretDirection);
        }

    }else{
        if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
            int extendTime = 750;
            catapult->charge_state = true;
            pros::delay(20);
            turret->rotateback(); 
            pros::delay(100);
            spool->moveTo(EXTENDED);
		    pros::delay(extendTime);
		    spool->moveTo(RETRACTED);
		    pros::delay(extendTime);
            turret->updatePosition();
            pros::delay(100);
            catapult->shoot_state = true; 
            catapult->shoot_ready = false;    
        }
    }    
    //rapidshootmode = (driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) ? !rapidshootmode : rapidshootmode;
    rapidshootmode = false;

    //pros::lcd::clear();
}