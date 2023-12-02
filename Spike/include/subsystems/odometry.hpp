#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "misc.hpp"
#include "drivetrain.hpp"
#include "pros/misc.hpp"

class Odometry
{
    public:
        Odometry(std::uint8_t gyro_port, std::uint8_t xTop, std::uint8_t xBottom, std::uint8_t yTop, std::uint8_t yBottom);
        Coordinate getPosition();
        void setPosition(Coordinate c);
        
        double getAngle();
        void setAngle(double angle);

        void resetTo(Coordinate c);
        void reset();
        void update();

    private:
        void updatePosition();
        void updateVelocity();
        void transformAcceleration();

        pros::IMU gyro;
        pros::ADIEncoder x, y;
        Coordinate position;
};

class PIDController
{
    public:
        PIDController(PIDConstants pidc);
        void setPID(PIDConstants pidc);
        void goToTarget(TankDrivetrain& d, Coordinate target, 
                Odometry& odom, int timeout);
        void goToAngle(TankDrivetrain& t, double angle, 
                Odometry& odom, int timeout);

    private:
        PIDConstants pid;
};

#endif