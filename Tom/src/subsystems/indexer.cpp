#include "subsystems/indexer.hpp"

#include "pros/rtos.hpp"

Indexer::Indexer(pros::ADIDigitalOut& s, pros::ADIDigitalOut& st) : solenoid{s}, start{st}
{
    open = false;
}

void Indexer::openIntake(){
    start.set_value(1);
    open = true;
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