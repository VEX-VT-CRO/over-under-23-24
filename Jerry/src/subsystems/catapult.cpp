#include "subsystems/catapult.hpp"
#include <cmath>
#include "lemlib/chassis/odom.hpp"

Catapult::Catapult(pros::Motor* m, pros::Rotation& rotated) : motor{m},rotate{rotated}
{
    motor->set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
    charge_state = false;
    shoot_state = false;
    shoot_ready = false;
    free_move = false;
    motor->set_encoder_units(pros::motor_encoder_units_e::E_MOTOR_ENCODER_DEGREES);
    rotate.set_position(35999);
    triggered = 0;
}


void Catapult::charge() {
    if (rotate.get_position()>29320)
        motor->move_voltage(-9000);
    else{
        charge_state = false;
        shoot_ready = true;
        motor->move_voltage(0);
    }
}

void Catapult::half()
{
    while(rotate.get_position()>31500)
    {
        motor->move_voltage(-9000);
    }
    motor->move_voltage(0);
}


void Catapult::spin(int mV)
{
    motor->move_voltage(mV);
}

void Catapult::shoot(){
    motor->move_relative(-70, 300);
    triggered++;
    shoot_state = false;
}