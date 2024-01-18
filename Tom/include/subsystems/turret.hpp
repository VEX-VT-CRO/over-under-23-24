#ifndef TURRET_HPP
#define TURRET_HPP

#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "misc.hpp"
#include "lemlib/api.hpp"

class Turret
{
    public:
        Turret(pros::Motor& motor1, pros::Motor& motor2, pros::IMU& gyro, PIDConstants PID);

        //When in autoAim mode, updatePosition does what it says
        void updatePosition(lemlib::Pose targetpos, lemlib::Pose currentpos);
        
        //Specifies how much the turret should be powered
        void turnVoltage(int mV);
        //Turns the turret by a specific amount of angles
        void turnAngle(int degrees);
        //Turns the turret to a particular point on the field (blocking function)
        void aimAt(Coordinate target, Coordinate pos);

    private:
        pros::Motor& turretMotor1, turretMotor2;
        PIDConstants kPID;
        pros::IMU& imu;

        int prevError, totalError;
};

#endif