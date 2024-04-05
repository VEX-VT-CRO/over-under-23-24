#ifndef INDEXER_HPP
#define INDEXER_HPP

#include "pros/adi.hpp"

class Indexer
{
    public:
        Indexer(pros::ADIDigitalOut& s1, pros::ADIDigitalOut& s2, pros::ADIDigitalOut& s3, pros::ADIDigitalOut& s4, pros::ADIDigitalOut& s5);
        void openFront();
        void openBack();
        void openOdometry();
    private:
        pros::ADIDigitalOut& back_right_solenoid;
        pros::ADIDigitalOut& back_left_solenoid;
        pros::ADIDigitalOut& front_right_solenoid;
        pros::ADIDigitalOut& front_left_solenoid;
        pros::ADIDigitalOut& odometry_solenoid;
        bool open_odometry;
        bool open_front;
        bool open_back;
};

#endif