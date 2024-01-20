#include "subsystems/indexer.hpp"

#include "pros/rtos.hpp"

Indexer::Indexer(pros::ADIDigitalOut& s) : solenoid{s}
{
    open = false;
}

void Indexer::indexDisc()
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