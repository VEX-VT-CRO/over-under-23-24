#include "subsystems/turret.hpp"
#include <cmath>
#include "lemlib/api.hpp"

Turret::Turret(pros::Motor& motor1, pros::Motor& motor2, pros::IMU& gyro, PIDConstants PID) : turretMotor1{motor1}, turretMotor2{motor2}, kPID{PID}, imu{gyro}
{
    prevError = 0;
    totalError = 0;
}


void Turret::turnVoltage(int mV)
{
    turretMotor1.move_voltage(mV);
    turretMotor2.move_voltage(mV);
}

void Turret::turnAngle(int degrees)
{
    turretMotor1.move_voltage(1.2*degrees);
    turretMotor2.move_voltage(1.2*degrees);
}

void Turret::aimAt(Coordinate target, Coordinate pos)
{
    //TODO: Place all of this in a loop

    //Step 1: Have a target (the goal), this is the parameter
    //Step 2: Find the angle to the target relative to the robot/current turret angle
    double goalAngle = DEG2RAD * atan2(target.y - pos.y, target.x - pos.x);
    double error = goalAngle - imu.get_heading();

    double derivError = error - prevError;
    totalError += error;
    //Step 3: Move a particular direction based on angle difference (using PID)

    double power = kPID.P * error + kPID.I * totalError + kPID.D * derivError;

    turretMotor1.move_voltage(power * 110);
    turretMotor2.move_voltage(power * 110);

    prevError = error;
}

void Turret::updatePosition(lemlib::Pose targetpos, lemlib::Pose currentpos)
{
    //Step 1: Have a target (the goal), this is the parameter
    //Step 2: Find the angle to the target relative to the robot/current turret angle
    double goal_Angle = DEG2RAD * atan2(targetpos.y - currentpos.y, targetpos.x - currentpos.x);
    double pos_angle = targetpos.theta-currentpos.theta;
    double rotated_angle = 0;
    
    // double error = goalAngle - imu.get_heading();

    // double derivError = error - prevError;
    // totalError += error;
    //Step 3: Move a particular direction based on angle difference (using PID)

    // double power = kPID.P * error + kPID.I * totalError + kPID.D * derivError;

    // turretMotor1.move_voltage(power * 110);
    // turretMotor2.move_voltage(power * 110);

    // prevError = error;
}