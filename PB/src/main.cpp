#include "main.h"

#include "lemlib/api.hpp"
#include "subsystems/rollerintake.hpp"
#include "subsystems/indexer.hpp"
#include "subsystems/climb.hpp"

//First robot to push balls
#define PB
//Second robot to push balls
// #define J

// #define QUAL_AUTO
#define MATCH_AUTO

// #define ARCADE
#define TANK

enum class RobotState {
    Driving,
    Intaking,
    Climbing
};

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

    constexpr int8_t HORIZONTAL_POD_PORT = 16;
    constexpr int8_t VERTICAL_POD_PORT = 15;
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

constexpr char BACK_LEFT_SOLENOID = 'A';
constexpr char BACK_RIGHT_SOLENOID = 'B';
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
    pros::Motor climb1(CLIMB_1_PORT);
    pros::Motor climb2(CLIMB_2_PORT, true);
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
pros::Rotation horizontalPod(HORIZONTAL_POD_PORT, true);
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
        1, // proportional gain (kP)
        0, // integral gain (kI)
        0, // derivative gain (kD)
        3, // anti windup
        1, // small error range, in inches
        100, // small error range timeout, in milliseconds
        3, // large error range, in inches
        500, // large error range timeout, in milliseconds
        20 // maximum acceleration (slew)
    );

    lemlib::ControllerSettings angularController(
        4, // proportional gain (kP)
        0, // integral gain (kI)
        42, // derivative gain (kD)
        3, // anti windup
        1, // small error range, in degrees
        100, // small error range timeout, in milliseconds
        3, // large error range, in degrees
        500, // large error range timeout, in milliseconds
        0 // maximum acceleration (slew)
    );
#elif defined(J)
    lemlib::ControllerSettings linearController(
        50, // proportional gain (kP)
        0, // integral gain (kI)
        175, // derivative gain (kD)
        3, // anti windup
        1, // small error range, in inches
        100, // small error range timeout, in milliseconds
        3, // large error range, in inches
        500, // large error range timeout, in milliseconds
        20 // maximum acceleration (slew)
    );

    lemlib::ControllerSettings angularController(
        4, // proportional gain (kP)
        0, // integral gain (kI)
        42, // derivative gain (kD)
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
    // nullptr,
    nullptr,
    &horizontalWheel,
    // nullptr,
    nullptr,
    &gyro
    // nullptr
);

// lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
//                                      10, // minimum output where drivetrain will move out of 127
//                                      1.019 // expo curve gain
// );

// lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
//                                   10, // minimum output where drivetrain will move out of 127
//                                   1.019 // expo curve gain
// );

// lemlib::Chassis chassis(LLDrivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);
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

void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();

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
            pros::lcd::print(6, "Robot State: %s", toString(robotState));        
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

void pollController()
{
    if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
    {
        ri.spin(ri.STANDARD_MV);
        if (robotState != RobotState::Intaking){
            robotState = RobotState::Intaking;
            setcurrentstate(robotState);
        }
    }
    else if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
    {
        ri.spin(-ri.STANDARD_MV);
        if (robotState != RobotState::Intaking){
            robotState = RobotState::Intaking;
            setcurrentstate(robotState);
        }
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
        climb.moveClimb(-12000);
        if (robotState != RobotState::Climbing){
            robotState = RobotState::Climbing;
            setcurrentstate(robotState);
        }
    }
    else if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
    {
        climb.moveClimb(12000);
        if (robotState != RobotState::Climbing){
            robotState = RobotState::Climbing;
            setcurrentstate(robotState);
        }
    }
    else
    {
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

ASSET(path_txt);
ASSET(pathJ_1_txt);
ASSET(pathJ_2_txt);

void qualPB()
{
    chassis.setPose({-50.5, -55, 125});
    for (int i = 0; i < 4; i++){
        leftSide[i].set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        rightSide[i].set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    }
    pros::delay(100);
    front_right_solenoid.set_value(1);
    pros::delay(500);
    front_right_solenoid.set_value(0);
    pros::delay(2000);
    front_right_solenoid.set_value(1);
    pros::delay(500);
    front_right_solenoid.set_value(0);
    pros::delay(2000);
    front_right_solenoid.set_value(1);
    pros::delay(500);
    front_right_solenoid.set_value(0);
    pros::delay(2000);
    front_right_solenoid.set_value(1);
    pros::delay(500);
    front_right_solenoid.set_value(0);
    pros::delay(2000);
    front_right_solenoid.set_value(1);
    pros::delay(500);
    front_right_solenoid.set_value(0);
    pros::delay(500);
    for (int i = 0; i < 4; i++){
        leftSide[i].set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        rightSide[i].set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    }
    front_left_solenoid.set_value(1);
    pros::delay(500);
    chassis.follow(path_txt, 15, 10000);
}

void matchPB()
{
    for(int i = 0; i < 4; i++)
    {
        leftSide[i].set_current_limit(2500);
        rightSide[i].set_current_limit(2500);
        climbGroup[i].set_current_limit(0);
    }
    intake.set_current_limit(0);

    chassis.setPose({0, 0, 0});
	chassis.moveToPose(0, 24, 0, 10000);
}

void qualJ()
{
    chassis.setPose({0, 0, 0});
	chassis.moveToPose(0, -20, 0, 10000,{false});
}

void matchJ()
{
    chassis.setPose({-34, -64, 0});
    chassis.follow(pathJ_1_txt, 15, 1500);
    pros::delay(1500);
    ri.spin(12000);
    chassis.follow(pathJ_2_txt, 15, 1000);
    pros::delay(1000);
    ri.spin(0);
    chassis.turnToHeading(90, 400);
    pros::delay(400);
    ri.spin(-12000);
    chassis.moveToPose(-15, -12, 90, 1500);
    pros::delay(1500);
    ri.spin(0);
    // chassis.turnToHeading(180, 3000);
    // chassis.turnToHeading(0, 3000);
    // chassis.turnToHeading(180, 3000);
    // chassis.turnToHeading(0, 3000);
}

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
        pollController();
        int l = driver.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        #if defined(ARCADE)
            int r = driver.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		    chassis.arcade(l, r);
	    #elif defined(TANK)
            int r = driver.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
		    chassis.tank(l, r);
	    #endif
        pros::delay(10);
    }
}