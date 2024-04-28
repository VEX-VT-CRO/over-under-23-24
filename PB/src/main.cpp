#include "main.h"

#include "lemlib/api.hpp"
#include "lemlib/util.hpp"
#include "subsystems/rollerintake.hpp"
#include "subsystems/indexer.hpp"
#include "subsystems/climb.hpp"

//First robot to push balls
// #define PB
//Second robot to push balls
#define J

#define QUAL_AUTO
// #define MATCH_AUTO

// #define ARCADE
#define TANK

enum class RobotState {
    Driving,
    Intaking,
    Climbing
};

bool auto_climb_state = false;

#if defined(PB)
    constexpr int8_t FRONT_LEFT_PORT         = 1;
    constexpr int8_t MIDDLE_FRONT_LEFT_PORT  = 2;
    constexpr int8_t MIDDLE_BACK_LEFT_PORT   = 3;
    constexpr int8_t BACK_LEFT_PORT          = 4;
    constexpr int8_t FRONT_RIGHT_PORT        = 5;
    constexpr int8_t MIDDLE_FRONT_RIGHT_PORT = 6;
    constexpr int8_t MIDDLE_BACK_RIGHT_PORT  = 7;
    constexpr int8_t BACK_RIGHT_PORT         = 8;

    constexpr int8_t INTAKE_PORT = 9;

    constexpr int8_t CLIMB_1_PORT = 11;
    constexpr int8_t CLIMB_2_PORT = 12;
    constexpr int8_t CLIMB_3_PORT = 13;
    constexpr int8_t CLIMB_4_PORT = 14;

    constexpr int8_t HORIZONTAL_POD_PORT = 15;
    constexpr int8_t VERTICAL_POD_PORT = 16;
    constexpr int8_t GYRO_PORT = 17;
    constexpr int8_t BALL_DISTANCE_PORT = 18;

#elif defined(J)
    constexpr int8_t FRONT_LEFT_PORT         = 1;
    constexpr int8_t MIDDLE_FRONT_LEFT_PORT  = 2;
    constexpr int8_t MIDDLE_BACK_LEFT_PORT   = 15;
    constexpr int8_t BACK_LEFT_PORT          = 13;
    constexpr int8_t FRONT_RIGHT_PORT        = 10;
    constexpr int8_t MIDDLE_FRONT_RIGHT_PORT = 9;
    constexpr int8_t MIDDLE_BACK_RIGHT_PORT  = 16;
    constexpr int8_t BACK_RIGHT_PORT         = 18;

    constexpr int8_t INTAKE_PORT = 6;

    constexpr int8_t CLIMB_1_PORT = 11;
    constexpr int8_t CLIMB_2_PORT = 20;

    constexpr int8_t HORIZONTAL_POD_PORT = 14;
    constexpr int8_t VERTICAL_POD_PORT = 17;
    constexpr int8_t GYRO_PORT = 5;
    constexpr int8_t BALL_DISTANCE_PORT = 3;
#endif    

constexpr double TRACK_WIDTH = 12;
constexpr double WHEEL_DIAMETER = 3;
constexpr double DRIVE_RPM = 600;
constexpr double CHASE_POWER = 2;

constexpr int32_t BALL_PRESENT_DISTANCE = 150;
constexpr int INTAKE_INTAKING_DIRECTION = 1;

constexpr double ODOM_WHEEL_DIAMETER = 2;
constexpr double HORIZONTAL_WHEEL_DISTANCE = 1.5625;
constexpr double VERTICAL_WHEEL_DISTANCE = -4.0625;

constexpr char BACK_LEFT_SOLENOID = 'B';
constexpr char BACK_RIGHT_SOLENOID = 'A';
constexpr char FRONT_LEFT_SOLENOID = 'C';
constexpr char FRONT_RIGHT_SOLENOID = 'D';
constexpr char ODOMETRY_SOLENOID = 'E';

pros::Controller driver(pros::controller_id_e_t::E_CONTROLLER_MASTER);

RobotState robotState = RobotState::Driving;

//DRIVETRAIN MOTORS

pros::Motor frontLeft(FRONT_LEFT_PORT);
pros::Motor middleFrontLeft(MIDDLE_FRONT_LEFT_PORT);
pros::Motor middleBackLeft(MIDDLE_BACK_LEFT_PORT);
pros::Motor backLeft(BACK_LEFT_PORT, true);
pros::Motor frontRight(FRONT_RIGHT_PORT, true);
pros::Motor middleFrontRight(MIDDLE_FRONT_RIGHT_PORT, true);
pros::Motor middleBackRight(MIDDLE_BACK_RIGHT_PORT, true);
pros::Motor backRight(BACK_RIGHT_PORT);
pros::Motor intake(INTAKE_PORT);
#if defined(PB)
    pros::Motor climb1(CLIMB_1_PORT);
    pros::Motor climb2(CLIMB_2_PORT);
    pros::Motor climb3(CLIMB_3_PORT, true);
    pros::Motor climb4(CLIMB_4_PORT, true);
#elif defined(J)
    pros::Motor climb1(CLIMB_1_PORT, true);
    pros::Motor climb2(CLIMB_2_PORT);
#endif

pros::ADIDigitalOut back_right_solenoid(BACK_RIGHT_SOLENOID);
pros::ADIDigitalOut back_left_solenoid(BACK_LEFT_SOLENOID);
pros::ADIDigitalOut front_right_solenoid(FRONT_RIGHT_SOLENOID);
pros::ADIDigitalOut front_left_solenoid(FRONT_LEFT_SOLENOID);
pros::ADIDigitalOut odometry_solenoid(ODOMETRY_SOLENOID);

pros::Motor_Group leftSide({frontLeft, middleFrontLeft, middleBackLeft, backLeft});
pros::Motor_Group rightSide({frontRight, middleFrontRight, middleBackRight, backRight});
pros::Motor_Group riGroup({intake});

#if defined(PB)
    pros::Motor_Group climbGroup({climb1, climb2, climb3, climb4});
#elif defined(J)
    pros::Motor_Group climbGroup({climb1, climb2});
#endif

//SENSORS
pros::Rotation horizontalPod(HORIZONTAL_POD_PORT);
pros::Rotation verticalPod(VERTICAL_POD_PORT);
pros::IMU gyro(GYRO_PORT);
pros::Distance ballDistance(BALL_DISTANCE_PORT);

//LEMLIB STRUCTURES

lemlib::TrackingWheel horizontalWheel(&horizontalPod, ODOM_WHEEL_DIAMETER, HORIZONTAL_WHEEL_DISTANCE);
lemlib::TrackingWheel verticalWheel(&verticalPod, ODOM_WHEEL_DIAMETER, VERTICAL_WHEEL_DISTANCE);

lemlib::Drivetrain LLDrivetrain(
    &leftSide,
    &rightSide,
    TRACK_WIDTH,
    WHEEL_DIAMETER,
    DRIVE_RPM,
    CHASE_POWER
);

#if defined(PB)
    lemlib::ControllerSettings linearController(
        10, // proportional gain (kP)
        0, // integral gain (kI)
        120, // derivative gain (kD)
        3, // anti windup
        1, // small error range, in inches
        100, // small error range timeout, in milliseconds
        3, // large error range, in inches
        500, // large error range timeout, in milliseconds
        10 // maximum acceleration (slew)
    );

    lemlib::ControllerSettings angularController(
        6, // proportional gain (kP)
        0, // integral gain (kI)
        50, // derivative gain (kD)
        3, // anti windup
        1, // small error range, in degrees
        100, // small error range timeout, in milliseconds
        3, // large error range, in degrees
        500, // large error range timeout, in milliseconds
        0 // maximum acceleration (slew)
    );
#elif defined(J)
    lemlib::ControllerSettings linearController(
        10, // proportional gain (kP)
        0, // integral gain (kI)
        120, // derivative gain (kD)
        3, // anti windup
        1, // small error range, in inches
        100, // small error range timeout, in milliseconds
        3, // large error range, in inches
        500, // large error range timeout, in milliseconds
        10 // maximum acceleration (slew)
    );

    lemlib::ControllerSettings angularController(
        2, // proportional gain (kP)
        0, // integral gain (kI)
        10, // derivative gain (kD)
        3, // anti windup
        1, // small error range, in degrees
        100, // small error range timeout, in milliseconds
        3, // large error range, in degrees
        500, // large error range timeout, in milliseconds
        0 // maximum acceleration (slew)
    );
#endif    

lemlib::OdomSensors sensors(
    &verticalWheel,
    nullptr,
    &horizontalWheel,
    nullptr,
    &gyro
);

lemlib::Chassis chassis(LLDrivetrain, linearController, angularController, sensors);

RollerIntake ri(riGroup);
Indexer ind(back_right_solenoid, back_left_solenoid, front_right_solenoid, front_left_solenoid, odometry_solenoid);
Climb climb(climbGroup);
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

const char* toString(RobotState state) {
    switch(state) {
        case RobotState::Driving:
            return "Driving";
        case RobotState::Intaking:
            return "Intaking";
        case RobotState::Climbing:
            return "Climbing";
        default:
            return "Unknown State";
    }
}

void setcurrentstate(RobotState state)
{

    if (state == RobotState::Driving)
    {
        for(int i = 0; i < 4; i++)
        {
            leftSide[i].set_current_limit(2500);
            rightSide[i].set_current_limit(2500);
        }
        intake.set_current_limit(0);
        #if defined(PB)
            for(int i = 0; i < 4; i++)
            {
                climbGroup[i].set_current_limit(0);
            }
        #elif defined(J)
            for(int i = 0; i < 2; i++)
            {
                climbGroup[i].set_current_limit(0);
            }
        #endif        
    }

    if (state == RobotState::Intaking)
    {
        for(int i = 0; i < 4; i++)
        {
            leftSide[i].set_current_limit(2200);
            rightSide[i].set_current_limit(2200);
        }
        intake.set_current_limit(2400);
        #if defined(PB)
            for(int i = 0; i < 4; i++)
            {
                climbGroup[i].set_current_limit(0);
            }
        #elif defined(J)
            for(int i = 0; i < 2; i++)
            {
                climbGroup[i].set_current_limit(0);
            }
        #endif
    }

    if (state == RobotState::Climbing)
    {
        intake.set_current_limit(0);
        #if defined(PB)
            for(int i = 0; i < 4; i++)
            {
                climbGroup[i].set_current_limit(2500);
            }
            for(int i = 0; i < 4; i++)
            {
                leftSide[i].set_current_limit(1250);
                rightSide[i].set_current_limit(1250);
            }
        #elif defined(J)
            for(int i = 0; i < 2; i++)
            {
                climbGroup[i].set_current_limit(2500);
            }
            for(int i = 0; i < 4; i++)
            {
                leftSide[i].set_current_limit(1875);
                rightSide[i].set_current_limit(1875);
            }
        #endif
    }
}

void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();

    leftSide.set_brake_modes(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
    rightSide.set_brake_modes(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);

    pros::Task screenTask([&]() {
    chassis.setPose({0, 0, 0});
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            #if defined(PB)
                pros::lcd::print(3, "PB");
            #elif defined(J)
                pros::lcd::print(3, "J");
            #endif
            #if defined(QUAL_AUTO)
                pros::lcd::print(4, "QUAL");
            #elif defined(MATCH_AUTO)
                pros::lcd::print(4, "MATCH");
            #endif
            pros::lcd::print(5, "Robot State: %s", toString(robotState));        
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            pros::delay(50);
        }
    });
}

void autoIntakeManager()
{
    while(!pros::competition::is_autonomous())
    {
        pros::delay(10);
    }
    while(pros::competition::is_autonomous())
    {
        if(ballDistance.get() < BALL_PRESENT_DISTANCE && riGroup.get_directions()[0] == INTAKE_INTAKING_DIRECTION)
        {
            riGroup.move(0);
        }

        pros::delay(10);
    }
}

//Link to curve
//https://www.desmos.com/calculator/umicbymbnl
double deadband = 0;
double minOutput = 0;
float expCurve(float input, float curveGain) {
    // return 0 if input is within deadzone
    if (fabs(input) <= deadband) return 0;
    // g is the output of g(x) as defined in the Desmos graph
    const float g = fabs(input) - deadband;
    // g127 is the output of g(127) as defined in the Desmos graph
    const float g127 = 127 - deadband;
    // i is the output of i(x) as defined in the Desmos graph
    const float i = pow(curveGain, g - 127) * g * lemlib::sgn(input);
    // i127 is the output of i(127) as defined in the Desmos graph
    const float i127 = pow(curveGain, g127 - 127) * g127;
    return (127.0 - minOutput) / (127) * i * 127 / i127 + minOutput * lemlib::sgn(input);
}


#if defined(PB)
void pollController()
{  
    if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
    {
        if (robotState != RobotState::Intaking){
            robotState = RobotState::Intaking;
            setcurrentstate(robotState);
        }
        ri.spin(ri.STANDARD_MV);
    }
    else if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
    {
        if (robotState != RobotState::Intaking){
            robotState = RobotState::Intaking;
            setcurrentstate(robotState);
        }
        ri.spin(-ri.STANDARD_MV);
    }
    else
    {
        ri.spin(0);
    }

    if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
        ind.openFront();
    }    
    if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
        ind.openBack();
    }
    if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
        ind.openOdometry();
    }

    if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_UP))
    {
         if (robotState != RobotState::Climbing){
            robotState = RobotState::Climbing;
            setcurrentstate(robotState);
        }
        climb.moveClimb(12000);
    }
    else if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
    {
        if (robotState != RobotState::Climbing){
            robotState = RobotState::Climbing;
            setcurrentstate(robotState);
        }
        climb.moveClimb(-12000);
    }
    else if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
    {
        if (robotState != RobotState::Climbing){
            robotState = RobotState::Climbing;
            setcurrentstate(robotState);
        }
        climb.deployClimb_PB();
        auto_climb_state = true;
    }
    else{
        climb.moveClimb(0);
    }

    if (!driver.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && 
        !driver.get_digital(pros::E_CONTROLLER_DIGITAL_L2) &&
        !driver.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && 
        !driver.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
    {
        if (robotState != RobotState::Driving){
            robotState = RobotState::Driving;
            setcurrentstate(robotState);
        }
    }
}
#elif defined(J)
void pollController()
{ 
    if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
    {
        if (robotState != RobotState::Intaking){
            robotState = RobotState::Intaking;
            setcurrentstate(robotState);
        }
        ri.spin(ri.STANDARD_MV);
    }
    else if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
    {
        if (robotState != RobotState::Intaking){
            robotState = RobotState::Intaking;
            setcurrentstate(robotState);
        }
        ri.spin(-ri.STANDARD_MV);
    }
    else
    {
        ri.spin(0);
    }

    if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
    {
        ind.openFrontLeft();
    }
    if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
    {
        ind.openFrontRight();
    }

    if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
        ind.openBack();
    }

    if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_A))
    {
        if (robotState != RobotState::Climbing){
            robotState = RobotState::Climbing;
            setcurrentstate(robotState);
        }
        climb.moveClimb(-12000);
    }
    else if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
    {
         if (robotState != RobotState::Climbing){
            robotState = RobotState::Climbing;
            setcurrentstate(robotState);
        }
        climb.moveClimb(12000);
    }
    else if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2))
    {
        if (robotState != RobotState::Climbing){
            robotState = RobotState::Climbing;
            setcurrentstate(robotState);
        }
        climb.deployClimb_J();  
        auto_climb_state = true;
    }
    else
    {
        climb.moveClimb(0);
    }

    if (!driver.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && 
        !driver.get_digital(pros::E_CONTROLLER_DIGITAL_L2) &&
        !driver.get_digital(pros::E_CONTROLLER_DIGITAL_A) && 
        !driver.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
    {
        if (robotState != RobotState::Driving){
            robotState = RobotState::Driving;
            setcurrentstate(robotState);
        }
    }
}
#endif

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

ASSET(PB_M_1_txt);
ASSET(PB_M_2_txt);
ASSET(PB_M_3_txt);
ASSET(PB_M_4_txt);
ASSET(PB_M_3_1_txt);
ASSET(PB_M_1_1_txt);

ASSET(J_M_1_txt);
ASSET(J_M_2_txt);

ASSET(J_Q_1_txt);
ASSET(J_Q_2_txt);
ASSET(J_Q_3_txt);

#if defined(PB)
void qualPB()
{
    chassis.setPose({-50.5, -55, 125});

    // //Wait for J to leave
    pros::delay(1000);

    //Release the intake and out-take balls
    climb.moveClimb(12000);
    front_left_solenoid.set_value(1);
    pros::delay(100);
    climb.moveClimb(-12000);
    pros::delay(200);
    climb.moveClimb(0);
    ri.spin(-ri.STANDARD_MV);
    pros::delay(500);

    //Prevent wheels from moving laterally
    for (int i = 0; i < 4; i++){
        leftSide[i].set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        rightSide[i].set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    }

    //Flick 12 triballs
    //Flick 11 in a loop
    constexpr int MAX_FLICKS = 12;
    for(int i = 0; i < MAX_FLICKS - 1; ++i)
    {
        front_right_solenoid.set_value(1);
        pros::delay(500);
        front_right_solenoid.set_value(0);
        pros::delay(1500);
    }
    //Flick one more
    front_right_solenoid.set_value(1);
    pros::delay(500);
    front_right_solenoid.set_value(0);
    //Retract left wing
    front_left_solenoid.set_value(0);
    pros::delay(500);

    //Let the chassis brake
    leftSide.set_brake_modes(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
    rightSide.set_brake_modes(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);

    //Push balls into alley
    chassis.follow(PB_M_1_1_txt, 30, 5000, true, false);
    pros::delay(10);
    leftSide.brake();
    rightSide.brake();
    pros::delay(500);
    
    //Back up for a spin
    chassis.follow(PB_M_2_txt, 30, 5000, false, false);
    pros::delay(10);
    leftSide.brake();
    rightSide.brake();
    ri.spin(0);
    pros::delay(500);

    //Turn around
    chassis.turnToHeading(0, 3000, true);
    chassis.turnToHeading(-90, 3000, false);
    pros::delay(500);

    //Push all the balls into the goal
    chassis.follow(PB_M_3_1_txt, 30, 6000, false, true);
    pros::delay(950);
    back_right_solenoid.set_value(1);
    // pros::delay(900);
    // back_left_solenoid.set_value(1);
    // pros::delay(500);
    // back_left_solenoid.set_value(0);
    // pros::delay(3550);
    pros::delay(5050);
    pros::delay(10);
    leftSide.brake();
    rightSide.brake();

    //Ram once
    // leftSide.move(50);
    // rightSide.move(50);
    // pros::delay(500);
    // leftSide.move(-127);
    // rightSide.move(-107);
    // pros::delay(1000);
    // leftSide.brake();
    // rightSide.brake();
    // pros::delay(200);

    //Drive away from goal to post
    chassis.setPose(59.728, -30.914, 180);
    //chassis.moveToPoint(59.728, -40.104, 10000, {}, false);
    leftSide.move(90);
    rightSide.move(90);
    pros::delay(300);
    leftSide.brake();
    rightSide.brake();
    pros::delay(200);
    back_right_solenoid.set_value(0);
    pros::delay(200);
    chassis.turnToHeading(270, 1000, false);
    ri.spin(12000);
    chassis.moveToPoint(22, -36.104, 2000, {}, false);
    chassis.turnToHeading(195, 1000, false);
    
    front_right_solenoid.set_value(1);
    leftSide.move(40);
    rightSide.move(40);
    pros::delay(1000);
    leftSide.brake();
    rightSide.brake();
    ri.spin(0);

    //Done, prepare the motors for driving
    for (int i = 0; i < 4; i++){
        leftSide[i].set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        rightSide[i].set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    }
}

void matchPB()
{   
    qualPB();
}

#elif defined(J)

void qualJ()
{
    chassis.setPose({-11.698, -62.532, 90});

    ri.spin(12000);

    chassis.follow(J_Q_1_txt, 30, 5000, true, false);
    leftSide.brake();
    rightSide.brake();
    pros::delay(500);

    ri.spin(-12000);
    pros::delay(500);
    front_right_solenoid.set_value(1);
    pros::delay(500);
    front_right_solenoid.set_value(0);
    pros::delay(500);

    chassis.follow(J_Q_2_txt, 30, 5000, true, false);
    leftSide.brake();
    rightSide.brake();
    pros::delay(500);

    ri.spin(0);
    leftSide.move(-100);
    rightSide.move(-100);
    pros::delay(100);
    leftSide.brake();
    rightSide.brake();
    pros::delay(200);
    chassis.turnToHeading(-90, 2000, false);
    leftSide.move(100);
    rightSide.move(100);
    pros::delay(550);
    leftSide.brake();
    rightSide.brake();
    pros::delay(200);
    chassis.turnToHeading(-180, 2000, false);
    leftSide.brake();
    rightSide.brake();
    pros::delay(200);
    leftSide.move(-80);
    rightSide.move(-80);
    pros::delay(600);
    leftSide.brake();
    rightSide.brake();

    pros::delay(30000);

    leftSide.move(80);
    rightSide.move(80);
    pros::delay(1200);
    leftSide.brake();
    rightSide.brake();
    pros::delay(200);
    chassis.turnToHeading(270, 2000, false);
    leftSide.move(100);
    rightSide.move(100);
    pros::delay(600);
    leftSide.brake();
    rightSide.brake();
}

void matchJ()
{
    chassis.setPose({-34, -64, 0});
    pros::delay(100);
    chassis.moveToPoint(-34, -38, 3000);
    pros::delay(3000);
    chassis.turnToHeading(90, 2000);
    pros::delay(2000);
    chassis.moveToPoint(-25, -40, 2000);
    pros::delay(2000);
    pros::delay(27000);
    chassis.moveToPoint(-65, -40, 3000, {false});
    pros::delay(3000);
    chassis.turnToHeading(-30, 2000);
    pros::delay(2000);
    back_left_solenoid.set_value(1);
    chassis.setPose({-54, -40.4, -30});
    chassis.follow(J_M_1_txt, 30, 5000, {false});
    pros::delay(5000);
    chassis.setPose({36, -60.5, 260});
    chassis.turnToHeading(225, 2000);
    pros::delay(2000);
    chassis.follow(J_M_2_txt, 20, 2500, {false});
    pros::delay(2500);
    chassis.turnToHeading(175, 2000);
    back_right_solenoid.set_value(1);
    pros::delay(2000);
    pros::delay(500);
    chassis.setPose({58, -36, 180});
    chassis.moveToPoint(58, -12, 1500, {false, 127, 100});
    pros::delay(1500);
    chassis.moveToPoint(58, -36, 1500);
    pros::delay(1500);
    chassis.moveToPoint(58, -12, 1500, {false, 127, 100});
    pros::delay(1500);
    pros::delay(500);
    back_left_solenoid.set_value(0);
    back_right_solenoid.set_value(0);
    chassis.setPose({0, 0, 0});
    pros::delay(250);
    chassis.moveToPoint(0, 15, 2000);
    pros::delay(2000);
    chassis.turnToHeading(45, 2000);
    pros::delay(2000);
    chassis.moveToPoint(15, 30, 2000);
    pros::delay(2000);
    chassis.turnToHeading(90, 2000);
    pros::delay(2000);
    chassis.moveToPoint(50, 30, 3000);
    pros::delay(3000);
}
#endif

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    pros::Task intakeTask(autoIntakeManager);

#if defined(PB)

	#if defined(QUAL_AUTO)
		qualPB();
	#elif defined(MATCH_AUTO)
		matchPB();
	#endif

#elif defined(J)

	#if defined(QUAL_AUTO)
		qualJ();
	#elif defined(MATCH_AUTO)
		matchJ();
	#endif

#endif
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    while(true)
    {
        if(!auto_climb_state)
            pollController();
        else
            if(abs(climbGroup[0].get_position() - climbGroup[0].get_target_position()) < 0.5)   
                auto_climb_state = false; 

#if defined(PB)
        if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
        {
            auto_climb_state = false;
        }
#elif defined(J)
        if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2))
        {
            auto_climb_state = false;
        }
#endif

        int l = driver.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
#if defined(ARCADE)
        int r = driver.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        r = expCurve(r, 1.015);
        chassis.arcade(l, r);
#elif defined(TANK)
        int r = driver.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        chassis.tank(l, r);
#endif

        pros::delay(10);
    }
}