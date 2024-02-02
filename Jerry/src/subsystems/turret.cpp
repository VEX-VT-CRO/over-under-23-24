#include "subsystems/turret.hpp"
#include <cmath>
#include "lemlib/api.hpp"

Turret::Turret(pros::Motor& motor1, pros::Motor& motor2, pros::Rotation& rotated, pros::IMU& gyro) : turretMotor1{motor1}, turretMotor2{motor2}, imu{gyro}, rotate{rotated}
{
   turretMotor1.set_encoder_units(pros::motor_encoder_units_e::E_MOTOR_ENCODER_DEGREES);
   turretMotor2.set_encoder_units(pros::motor_encoder_units_e::E_MOTOR_ENCODER_DEGREES);
   rotation = 0;
}


void Turret::turnVoltage(int mV)
{
    turretMotor1.move_voltage(mV);
    turretMotor2.move_voltage(mV);
}

void Turret::turnAngle(int degrees)
{
    turretMotor1.move_relative(-1.56*degrees, 200);
    turretMotor2.move_relative(-1.56*degrees, 200);
    rotation+=degrees;
    last_rotation = degrees;
}

void Turret::updatePosition(lemlib::Pose targetpos, lemlib::Pose currentpos)
{
    double goal_Angle = -DEG2RAD * atan2(targetpos.y - currentpos.y, targetpos.x - currentpos.x);
    double rotated_angle = imu.get_rotation();
    double turn_angle = goal_Angle + rotated_angle;
    Turret::turnAngle(turn_angle);
}

void Turret::checkRotation(){
    if (abs(rotation)>=1080){
        turretMotor1.move_relative(1.56*(rotation-last_rotation), 200);
        turretMotor2.move_relative(1.56*(rotation-last_rotation), 200);
        rotation = 0;
    }
}