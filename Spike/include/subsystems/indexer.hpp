#ifndef INDEXER_HPP
#define INDEXER_HPP

#include "pros/adi.hpp"

class Indexer
{
    public:
        Indexer(pros::ADIDigitalOut& solenoid);
        void indexDisc();
    private:
        pros::ADIDigitalOut& solenoid;
        bool open;
};

#endif