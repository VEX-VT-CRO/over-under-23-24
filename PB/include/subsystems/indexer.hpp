#ifndef INDEXER_HPP
#define INDEXER_HPP

#include "pros/adi.hpp"

class Indexer
{
    public:
        Indexer(pros::ADIDigitalOut& br, pros::ADIDigitalOut& bl, pros::ADIDigitalOut& fr, pros::ADIDigitalOut& fl, pros::ADIDigitalOut& odom);
        void openFront();
        void openBack();
        void openOdometry();

        void openFrontLeft();
        void openFrontRight();
        void openBackLeft();
        void openBackRight();
    private:
        pros::ADIDigitalOut& back_right_solenoid;
        pros::ADIDigitalOut& back_left_solenoid;
        pros::ADIDigitalOut& front_right_solenoid;
        pros::ADIDigitalOut& front_left_solenoid;
        pros::ADIDigitalOut& odometry_solenoid;
        bool open_front_left;
        bool open_front_right;
        bool open_back_left;
        bool open_back_right;
};

#endif