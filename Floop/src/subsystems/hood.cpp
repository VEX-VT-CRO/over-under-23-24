#include "subsystems/hood.hpp"

#include "pros/rtos.hpp"

Hood::Hood(pros::ADIDigitalOut& s) : solenoid{s}
{
    open = false;
}

void Hood::indexDisc()
{
    if(!open){
        solenoid.set_value(1);
        open = !open; 
    }
    else{
        solenoid.set_value(0);
        open = !open;
    }    
}        