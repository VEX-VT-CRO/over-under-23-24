#ifndef HOOD_HPP
#define HOOD_HPP

#include "pros/adi.hpp"

class Hood
{
    public:
        Hood(pros::ADIDigitalOut& solenoid);
        void indexDisc();
    private:
        pros::ADIDigitalOut& solenoid;
        bool open;
};

#endif