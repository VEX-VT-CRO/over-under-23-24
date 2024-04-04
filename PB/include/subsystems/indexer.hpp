#ifndef INDEXER_HPP
#define INDEXER_HPP

#include "pros/adi.hpp"

class Indexer
{
    public:
        Indexer(pros::ADIDigitalOut& s1, pros::ADIDigitalOut& s2, pros::ADIDigitalOut& s3, pros::ADIDigitalOut& s4, pros::ADIDigitalOut& s5);
        void indexDisc();
        void Odometry();
    private:
        pros::ADIDigitalOut& solenoid1;
        pros::ADIDigitalOut& solenoid2;
        pros::ADIDigitalOut& solenoid3;
        pros::ADIDigitalOut& solenoid4;
        pros::ADIDigitalOut& solenoid5;
        bool open;
        bool open_odometry;
};

#endif