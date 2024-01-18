#ifndef INDEXER_HPP
#define INDEXER_HPP

#include "pros/adi.hpp"

class Indexer
{
    public:
        Indexer(pros::ADIDigitalOut& solenoid);
        void indexDisc(bool toggle_pneumatics);
    private:
        pros::ADIDigitalOut& solenoid;
        bool open;
};

#endif