#ifndef VISIONSENSOR_HPP
#define VISIONSENSOR_HPP

#include "misc.hpp"
#include "api.h"
#include "pros/adi.hpp"

class VisionSensor
{
    public:
        VisionSensor(pros::Vision& vision_sensor);
        void target_position();
        void findObjects();
    private:
        Coordinate current_position;
        pros::Vision& vis;
        pros::vision_signature_s_t GREEN_SIG;
        pros::vision_object_s_t rtn;
};

#endif