#include "subsystems/indexer.hpp"
#include "pros/rtos.hpp"

// Use initializer list to initialize the solenoids and member variables
Indexer::Indexer(pros::ADIDigitalOut& s1, pros::ADIDigitalOut& s2, pros::ADIDigitalOut& s3, pros::ADIDigitalOut& s4, pros::ADIDigitalOut& s5) : back_right_solenoid(s1), back_left_solenoid(s2), front_right_solenoid(s3), front_left_solenoid(s4), odometry_solenoid(s5)
{
   open_front = false;
   open_back = false;
   open_odometry = false;
}

void Indexer::openFront()
{
    if (!open_front) {
        front_right_solenoid.set_value(1);
        front_left_solenoid.set_value(1);
        open_front = true; 
    } else {
        front_right_solenoid.set_value(0);
        front_left_solenoid.set_value(0);
        open_front = false;
    }
}

void Indexer::openBack()
{
    if (!open_back) {
        back_right_solenoid.set_value(1);
        back_left_solenoid.set_value(1);
        open_back = true; 
    } else {
        back_left_solenoid.set_value(0);
        back_right_solenoid.set_value(0);
        open_back = false;
    }
}

void Indexer::openOdometry()
{
    if (!open_odometry) {
        odometry_solenoid.set_value(1);
        open_odometry = true; 
    } else {
        odometry_solenoid.set_value(0);
        open_odometry = false;
    }
}
