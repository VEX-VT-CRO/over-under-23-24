#include "subsystems/indexer.hpp"

#include "pros/rtos.hpp"

Indexer::Indexer(pros::ADIDigitalOut& s) : solenoid{s}
{
    open = false;
}

void Indexer::indexDisc(bool toggle_pneumatics)
{
    if(!toggle_pneumatics){
        if(!open){
            solenoid.set_value(1);
             open = !open; 
       }
        else{
            solenoid.set_value(0);
            open = !open;
        }    
    }
    else{
        if(open){
            solenoid.set_value(0);
            pros::delay(20);
        }
        else{
            solenoid.set_value(1);
            pros::delay(500);
            solenoid.set_value(0);
        }     
    }
}        