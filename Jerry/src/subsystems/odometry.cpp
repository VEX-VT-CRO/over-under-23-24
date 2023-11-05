#include "subsystems/odometry.hpp"
#include "pros/rtos.hpp"
#include "pros/imu.h"
#include "pros/misc.hpp"
#include <float.h>
#include <cmath>

constexpr double TICKS_PER_REV = 360;
constexpr double ODOM_DIAMETER = 2.8; //in
constexpr double DT = 0.01; //s
constexpr double LINEAR_CONSTANT = PI * ODOM_DIAMETER / TICKS_PER_REV; //in/tick

constexpr double ABS(double value)
{
    return (value < 0) ? value * -1 : value;
}

Odometry::Odometry(std::uint8_t gyro_port, std::uint8_t xTop, std::uint8_t xBottom, std::uint8_t yTop, std::uint8_t yBottom) : 
        gyro{pros::IMU(gyro_port)}, x{pros::ADIEncoder(xTop, xBottom)}, y{pros::ADIEncoder(yTop, yBottom)}
{
    while(gyro.is_calibrating())
    {
        pros::delay(250);
    }

    x.reset();
    y.reset();
}

Coordinate Odometry::getPosition()
{
    return position;
}

void Odometry::setPosition(Coordinate c)
{
    position = c;
}

void Odometry::resetTo(Coordinate c)
{
    gyro.reset();
    position = c;
}

void Odometry::reset()
{
    gyro.reset();
    position = {0, 0, 0};
}

void Odometry::update()
{
    //TODO: Detect driving over the bar
    
    constexpr double TURN_THRESHOLD = 0.005;

    static double prevTheta = 0;
    double theta = gyro.get_rotation() * PI / 180; //In radians

    //Angle change (turning robot if non-zero)
    double changeInTheta = theta - prevTheta;
    prevTheta = theta;

    double changeX = (x.get_value() * LINEAR_CONSTANT); //the offset to apply to the x odometry wheel
    double changeY = (y.get_value() * LINEAR_CONSTANT); //the offset to apply to the y odometry wheel
    double sinChangeInTheta = 2 * sin(changeInTheta / 2);

    //Check if the robot turns at all
    if (ABS(changeInTheta) > TURN_THRESHOLD) {
        //Calculates largest double that would be able to be squared without throwing an error
        //dx is the horizontal distance to the center of rotation
        double radiusX = changeX / (changeInTheta) - 5.875;
        double radiusY = changeY / (changeInTheta) + 1.875; //The offset in the y-axis of the x omni-wheel

        changeX = radiusX * sinChangeInTheta;
        changeY = radiusY * sinChangeInTheta;
    }

    double costheta = cos(theta);
    double sintheta = sin(theta);

    position.x +=  costheta * changeX + sintheta * changeY;
    position.y += -sintheta * changeX + costheta * changeY;

    x.reset();
    y.reset();

    pros::lcd::print(0, "X POS: %f", position.x);
    pros::lcd::print(1, "Y POS: %f", position.y);
}

void Odometry::transformAcceleration() {
    /*double ax = getXAccel();
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
    XYangle = acos(r11 / magnitude);*/
}

void Odometry::updateVelocity() {
        /*velocity.x += floor(acceleration.x * DT * 10) / 10;
        velocity.y += floor(acceleration.y * DT * 10) / 10;
        velocity.z += floor(acceleration.z * DT * 10) / 10;*/
    }

void Odometry::updatePosition() {
    //position.x += floor(velocity.x * DT * 10) / 10 + floor(0.5 * acceleration.x * DT * 10) / 10;
    //position.y += floor(velocity.y * DT * 10) / 10 + floor(0.5 * acceleration.y * DT * 10) / 10;
    //position.z += floor(velocity.z * DT * 10) / 10 + floor(0.5 * acceleration.z * DT * 10) / 10;
    
    }

double Odometry::getAngle()
{
    return gyro.get_heading();
}

void Odometry::setAngle(double angle)
{
    gyro.set_heading(angle);
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
    double lowerBound = 10;
    bool useX = true;
    double goal;
    int sign;

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

    sign = (goal - ((useX) ? odom.getPosition().x : odom.getPosition().y)) < 0 ? -1 : 1;

    //Start the clock, the difference in start time and current time must
    //be less than the timeout to make sure we don't spend too much time
    //blocking the program execution.
    int timer = pros::millis();
    while(pros::millis() - timer < timeout)
    {
        odom.update();
        //This chooses what our measured value is, based on the conditional block above.
        double actual = (useX) ? odom.getPosition().x : odom.getPosition().y;
        double error = goal - actual;

        //This is the error threshold for what is acceptable as "close enough".
        if(std::abs(error) < 0.05)
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
        if(ABS(power) < lowerBound)
        {
            power = lowerBound * ((error < 0) ? -1 : 1);
        }

        //Power is multiplied into units of mV to provide more accurate control.
        d.drive(sign * power * 110);

        pros::lcd::print(0, "ERROR: %f", error);
        pros::lcd::print(1, "POWER: %f", power);

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
        double error = target - odom.getAngle();

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