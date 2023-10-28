#include "subsystems/odometry.hpp"
#include "pros/rtos.hpp"
#include "pros/imu.h"
#include "pros/misc.hpp"
#include <cmath>

constexpr double DT = 0.01; //s
constexpr double G  = 9.81; //m/s^2 

Odometry::Odometry(std::uint8_t xGyro_port, std::uint8_t yGyro_port, std::uint8_t zGyro_port) : 
        xGyro{pros::Imu(xGyro_port)}
{
    position = {0, 0, 0};
    velocity = {0, 0, 0};
    acceleration = {0, 0, 0};
    XYangle = 0.0;

    while(xGyro.is_calibrating())
    {
        pros::delay(250);
    }

    pros::delay(500);
    pros::c::imu_accel_s_t a = xGyro.get_accel();
    accelAtRest = {floor(a.x * G * 10) / 10, floor(a.y * G * 10) / 10, floor(a.z * G * 10) / 10};
    accelAtRest.x = (std::abs(accelAtRest.x) > 0.5) ? accelAtRest.x : 0;
    accelAtRest.y = (std::abs(accelAtRest.y) > 0.5) ? accelAtRest.y : 0;
    accelAtRest.z = (std::abs(accelAtRest.z) > 0.5) ? accelAtRest.z : 0;
}

double Odometry::getXYAngle()
{
    return XYangle;
}

Coordinate Odometry::getPosition()
{
    return position;
}

void Odometry::setPosition(Coordinate c)
{
    position = c;
}

void Odometry::reset()
{
    xGyro.reset();
}

Coordinate Odometry::getVelocity()
{
    return velocity;
}

Coordinate Odometry::getAcceleration()
{
    return acceleration;
}

void Odometry::update()
{
    transformAcceleration();
    updatePosition();
    updateVelocity();
}

void Odometry::transformAcceleration() {
    double ax = getXAccel();
    double ay = getYAccel();
    double az = getZAccel();
    
    double yaw = -getYaw();
    double pitch = -getPitch();
    double roll = -getRoll();

    double cosyaw = cos(yaw);
    double sinyaw = sin(yaw);
    double cospitch = cos(pitch);
    double sinpitch = sin(pitch);
    double cosroll = cos(roll);
    double sinroll = sin(roll);

    double r11 = cosyaw * cospitch;
    double r12 = cosyaw * sinpitch * sinroll - sinyaw * cosroll;
    double r13 = cosyaw * sinpitch * cosroll + sinyaw * sinroll;
    
    double r21 = sinyaw * cospitch;
    double r22 = sinyaw * sinpitch * sinroll + cosyaw * cosroll;
    double r23 = sinyaw * sinpitch * cosroll - cosyaw * sinroll;
    
    double r31 = -sinpitch;
    double r32 = cospitch * sinroll;
    double r33 = cospitch * cosroll;

    //acceleration.x = r11 * ax + r12 * ay + r13 * az - 0.1;
    //acceleration.y = r21 * ax + r22 * ay + r23 * az;
    //acceleration.z = r31 * ax + r32 * ay + r33 * az - GRAVITY;

    acceleration.x = ax - accelAtRest.x;
    acceleration.y = ay - accelAtRest.y;
    acceleration.z = az - accelAtRest.z;

    double magnitude = sqrt(r11 * r11 + r21 * r21);
    XYangle = acos(r11 / magnitude);
}

void Odometry::updateVelocity() {
        velocity.x += floor(acceleration.x * DT * 10) / 10;
        velocity.y += floor(acceleration.y * DT * 10) / 10;
        velocity.z += floor(acceleration.z * DT * 10) / 10;
    }

void Odometry::updatePosition() {
    position.x += floor(velocity.x * DT * 10) / 10 + floor(0.5 * acceleration.x * DT * 10) / 10;
    position.y += floor(velocity.y * DT * 10) / 10 + floor(0.5 * acceleration.y * DT * 10) / 10;
    position.z += floor(velocity.z * DT * 10) / 10 + floor(0.5 * acceleration.z * DT * 10) / 10;
    }

double Odometry::getYaw()
{
    return xGyro.get_heading();
}

double Odometry::getPitch()
{
    return xGyro.get_pitch();
}

double Odometry::getRoll()
{
    return xGyro.get_roll();
}

double Odometry::getXAccel()
{
    pros::c::imu_accel_s_t accel = xGyro.get_accel();
    double ax = accel.x * G;
    ax = floor(ax * 10) / 10;
    return (std::abs(ax) > 0.5) ? ax : 0;
}

double Odometry::getYAccel()
{
    pros::c::imu_accel_s_t accel = xGyro.get_accel();
    double ay = accel.y * G;
    ay = floor(ay * 10) / 10;
    return (std::abs(ay) > 0.5) ? ay : 0;
}

double Odometry::getZAccel()
{
    pros::c::imu_accel_s_t accel = xGyro.get_accel();
    double az = accel.z * G;
    az = floor(az * 10) / 10;
    return (std::abs(az) > 0.5) ? az : 0;
}

void Odometry::setPitch(double angle)
{
    xGyro.set_pitch(angle);
}

void Odometry::setYaw(double angle)
{
    xGyro.set_heading(angle);
}

void Odometry::setRoll(double angle)
{
    xGyro.set_roll(angle);
}

PIDController::PIDController(PIDConstants pidc)
{
    this->pid = pidc;
}

void PIDController::setPID(PIDConstants pidc)
{
    this->pid = pidc;
}

void PIDController::goToTarget(TankDrivetrain &d, Coordinate target, 
        Odometry &odom, int timeout)
{
    Coordinate pos = odom.getPosition();
    double prevError = 0;
    double nextError = 0;
    double lowerBound = 100;
    bool useX = true;
    double goal;

    //Since the target is on a 2D plane, figure out which dimension to use as the target,
    //rather than computing the absolute distance.
    if(std::abs(target.x - pos.x) > std::abs(target.y - pos.y))
    {
        goal = target.x;
    }
    else
    {
        goal = target.y;
        useX = false;
    }

    //Start the clock, the difference in start time and current time must
    //be less than the timeout to make sure we don't spend too much time
    //blocking the program execution.
    int timer = pros::millis();
    while(pros::millis() - timer < timeout)
    {
        //This chooses what our measured value is, based on the conditional block above.
        double actual = (useX) ? odom.getPosition().x : odom.getPosition().y;
        double error = goal - actual;

        //This is the error threshold for what is acceptable as "close enough".
        if(std::abs(error) < 0.1)
        {
            break;
        }

        //Derivative is a prediction of what will happen in the future.
        double derivError = error - prevError;

        //Integral calculation for what has happened in the past.
        nextError += error;

        //The magic happens here: the values are added together to determine
        //how much power the robot should receive to reach its goal.
        //We attenuate how much influence each factor has by multiplying PID
        //by a constant.
        double power = pid.P * error + pid.I * nextError + pid.D * derivError;

        //If the power is too low, the robot won't move and nothing happens.
        //Technically, integral can fix this on its own but that's a lot of work 
        //and introduces more issues than it solves.
        if(power < lowerBound)
        {
            power = lowerBound;
        }

        //Power is multiplied into units of mV to provide more accurate control.
        d.drive(power * 110);

        //Remember what the last error was for derivative calculations in the next iteration.
        prevError = error;
        pros::delay(10);
    }

    //Tell the robot to stop moving.
    d.drive(0);
}

void PIDController::goToAngle(TankDrivetrain &d, double target, 
        Odometry &odom, int timeout)
{
    double prevError = 0;
    double nextError = 0;
    double lowerBound = 100;

    int timer = pros::millis();
    while(pros::millis() - timer < timeout)
    {
        double error = target - odom.getYaw();

        if(std::abs(error) < 0.1)
        {
            break;
        }

        double derivError = error - prevError;
        nextError += error;

        double power = pid.P * error + pid.I * nextError + pid.D * derivError;

        if(power < lowerBound)
        {
            power = lowerBound;
        }

        d.turnLeft(power * 110);

        prevError = error;
        pros::delay(10);
    }

    d.turnLeft(0);
}