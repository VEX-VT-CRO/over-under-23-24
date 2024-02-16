#include "subsystems/catapult.hpp"
#include <cmath>
#include "lemlib/chassis/odom.hpp"

Catapult::Catapult(pros::Motor* m1, pros::Motor* m2, pros::ADIDigitalIn* catapult_charged, pros::Distance* distance_sensor){
    goal_position = {0,0,0};
    motor1 = m1;
    motor2 = m2;
    charged = catapult_charged;
    triball_in = distance_sensor;
    charged = catapult_charged;
    distance = 0;
    mV = 0;
    triball_distance = 50;
    charge_state = false;
    shoot_state = false;

    motor1->set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
    motor2->set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
}


void Catapult::charge() {
        if (charged->get_value()) {
            motor1->move_voltage(0);
            motor2->move_voltage(0);
            charge_state=false;
        }
        else{
            motor1->move_voltage(-12000);
            motor2->move_voltage(-12000);
        }
}

void Catapult::spin(int mV)
{
    motor1->move_voltage(mV);
    motor2->move_voltage(mV);
}