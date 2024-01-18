#ifndef INDEXER_HPP
#define INDEXER_HPP

#include "pros/adi.hpp"

class Indexer
{
    public:
        Indexer(pros::ADIDigitalOut& solenoid, pros::ADIDigitalOut& start);
        void indexDisc();
        void openIntake();
    private:
        pros::ADIDigitalOut& solenoid;
        pros::ADIDigitalOut& start;
        bool open;
};

#endif