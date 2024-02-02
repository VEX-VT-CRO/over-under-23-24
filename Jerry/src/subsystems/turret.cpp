#include "subsystems/turret.hpp"
#include <cmath>
#include "lemlib/api.hpp"

Turret::Turret(pros::Motor& motor1, pros::Motor& motor2, pros::Rotation& rotated, pros::IMU& gyro) : turretMotor1{motor1}, turretMotor2{motor2}, imu{gyro}, rotate{rotated}
{
}


void Turret::turnVoltage(int mV)
{
    turretMotor1.move_voltage(mV);
    turretMotor2.move_voltage(mV);
}

void Turret::turnAngle(int degrees)
{
    float radians = degrees * DEG2RAD;
    uint32_t startTime = pros::millis();
    while (true) {
        if (pros::millis() - startTime > 165*abs(radians)) {
            turretMotor1.move_voltage(0);
            turretMotor2.move_voltage(0);
            break;
        }
        turretMotor1.move_voltage(-6000*radians/abs(radians));
        turretMotor2.move_voltage(-6000*radians/abs(radians));
    }
    turretMotor1.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
    turretMotor2.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
}

void Turret::updatePosition(lemlib::Pose targetpos, lemlib::Pose currentpos)
{
    double goal_Angle = -DEG2RAD * atan2(targetpos.y - currentpos.y, targetpos.x - currentpos.x);
    double rotated_angle = imu.get_rotation();
    double turn_angle = goal_Angle + rotated_angle;
    Turret::turnAngle(turn_angle);
}