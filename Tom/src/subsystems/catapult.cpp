#include "subsystems/catapult.hpp"
#include <cmath>
#include "lemlib/chassis/odom.hpp"

Catapult::Catapult(pros::Motor* m, pros::ADIDigitalIn* catapult_charged, pros::Distance* distance_sensor){
    goal_position = {0,0,0};
    motor = m;
    charged = catapult_charged;
    triball_in = distance_sensor;
    charged = catapult_charged;
    distance = 0;
    mV = 0;
    triball_distance = 50;
    charge_state = false;
    shoot_state = false;

    motor->set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
}


void Catapult::charge() {
        if (charged->get_value()) {
            motor->move_voltage(0);
            charge_state=false;
        }
        else{
            motor->move_voltage(-6000);
        }
}

void Catapult::spin(int mV)
{
    motor->move_voltage(mV);
}