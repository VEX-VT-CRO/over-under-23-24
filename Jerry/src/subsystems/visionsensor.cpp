#include "subsystems/visionsensor.hpp"

VisionSensor::VisionSensor(pros::Vision& vision_sensor):vis{vision_sensor}
{
    //Just example numbers, need to be adjusted
    GREEN_SIG = pros::Vision::signature_from_utility(1,8973, 11143, 10058, -2119, -1053, -1586, 5.4, 0);
    vis.set_signature(1,&GREEN_SIG);
}

void VisionSensor::findObjects(){
    while (true) {
        // Gets the largest object of the first signature
    rtn = vis.get_by_sig(0, 1);
    pros::delay(2);
  }
}
void VisionSensor::target_position(){

}