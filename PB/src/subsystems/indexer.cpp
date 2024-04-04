#include "subsystems/indexer.hpp"
#include "pros/rtos.hpp"

// Use initializer list to initialize the solenoids and member variables
Indexer::Indexer(pros::ADIDigitalOut& s1, pros::ADIDigitalOut& s2, pros::ADIDigitalOut& s3, pros::ADIDigitalOut& s4, pros::ADIDigitalOut& s5) : solenoid1(s1), solenoid2(s2), solenoid3(s3), solenoid4(s4), solenoid5(s5)
{
   open = false;
   open_odometry = false;
}

void Indexer::indexDisc()
{
    if (!open) {
        solenoid1.set_value(1);
        open = true; 
    } else {
        solenoid1.set_value(0);
        open = false;
    }
}

void Indexer::Odometry()
{
    if (!open_odometry) {
        solenoid4.set_value(1);
        solenoid5.set_value(1);
        open_odometry = true; 
    } else {
        solenoid4.set_value(0);
        solenoid5.set_value(0);
        open_odometry = false;
    }
}
