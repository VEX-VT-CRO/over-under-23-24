#include "main.h"

#include "lemlib/api.hpp"
#include "subsystems/rollerintake.hpp"
#include "subsystems/indexer.hpp"

//First robot to push balls
#define PB
//Second robot to push balls
//#define J

#define QUAL_AUTO
//#define MATCH_AUTO

// #define ARCADE
#define TANK

constexpr int8_t FRONT_LEFT_PORT         = 1;
constexpr int8_t MIDDLE_FRONT_LEFT_PORT  = 2;
constexpr int8_t MIDDLE_BACK_LEFT_PORT   = 3;
constexpr int8_t BACK_LEFT_PORT          = 4;
constexpr int8_t FRONT_RIGHT_PORT        = 5;
constexpr int8_t MIDDLE_FRONT_RIGHT_PORT = 6;
constexpr int8_t MIDDLE_BACK_RIGHT_PORT  = 7;
constexpr int8_t BACK_RIGHT_PORT         = 8;

constexpr int8_t INTAKE_1_PORT = 9;
constexpr int8_t INTAKE_2_PORT = 10;

constexpr int8_t CLIMB_1_PORT = 11;
constexpr int8_t CLIMB_2_PORT = 12;
constexpr int8_t CLIMB_3_PORT = 13;
constexpr int8_t CLIMB_4_PORT = 14;

constexpr int8_t HORIZONTAL_POD_PORT = 15;
constexpr int8_t VERTICAL_POD_PORT = 16;
constexpr int8_t GYRO_PORT = 17;

constexpr int ODOM_WHEEL_DIAMETER = 2;
constexpr int HORIZONTAL_WHEEL_DISTANCE = 0;
constexpr int VERTICAL_WHEEL_DISTANCE = 0;

constexpr int TRACK_WIDTH = 12;
constexpr int WHEEL_DIAMETER = 3;
constexpr int DRIVE_RPM = 600;
constexpr int CHASE_POWER = 2;

constexpr char BACK_LEFT_SOLENOID = 'A';
constexpr char BACK_RIGHT_SOLENOID = 'B';
constexpr char FRONT_LEFT_SOLENOID = 'C';
constexpr char FRONT_RIGHT_SOLENOID = 'D';
constexpr char ODOMETRY_SOLENOID = 'E';

pros::Controller driver(pros::controller_id_e_t::E_CONTROLLER_MASTER);

//DRIVETRAIN MOTORS

pros::Motor frontLeft(FRONT_LEFT_PORT);
pros::Motor middleFrontLeft(MIDDLE_FRONT_LEFT_PORT);
pros::Motor middleBackLeft(MIDDLE_BACK_LEFT_PORT);
pros::Motor backLeft(BACK_LEFT_PORT, true);
pros::Motor frontRight(FRONT_RIGHT_PORT, true);
pros::Motor middleFrontRight(MIDDLE_FRONT_RIGHT_PORT, true);
pros::Motor middleBackRight(MIDDLE_BACK_RIGHT_PORT, true);
pros::Motor backRight(BACK_RIGHT_PORT);
pros::Motor intake1(INTAKE_1_PORT);
pros::Motor intake2(INTAKE_2_PORT, true);
pros::Motor climb1(CLIMB_1_PORT);
pros::Motor climb2(CLIMB_2_PORT);
pros::Motor climb3(CLIMB_3_PORT, true);
pros::Motor climb4(CLIMB_4_PORT, true);
pros::ADIDigitalOut back_right_solenoid(BACK_RIGHT_SOLENOID);
pros::ADIDigitalOut back_left_solenoid(BACK_LEFT_SOLENOID);
pros::ADIDigitalOut front_right_solenoid(FRONT_RIGHT_SOLENOID);
pros::ADIDigitalOut front_left_solenoid(FRONT_LEFT_SOLENOID);
pros::ADIDigitalOut odometry_solenoid(ODOMETRY_SOLENOID);

pros::Motor_Group leftSide({frontLeft, middleFrontLeft, middleBackLeft, backLeft});
pros::Motor_Group rightSide({frontRight, middleFrontRight, middleBackRight, backRight});
pros::MotorGroup riGroup({intake1, intake2});
pros::MotorGroup climbGroup({climb1, climb2, climb3, climb4});
//SENSORS

pros::Rotation horizontalPod(HORIZONTAL_POD_PORT);
pros::Rotation verticalPod(VERTICAL_POD_PORT);
pros::IMU gyro(GYRO_PORT);

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

lemlib::ControllerSettings linearController(
    10, // proportional gain (kP)
    0, // integral gain (kI)
    3, // derivative gain (kD)
    3, // anti windup
    1, // small error range, in inches
    100, // small error range timeout, in milliseconds
    3, // large error range, in inches
    500, // large error range timeout, in milliseconds
    20 // maximum acceleration (slew)
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

lemlib::OdomSensors sensors(
    //&verticalWheel,
    nullptr,
    nullptr,
    //&horizontalWheel,
    nullptr,
    nullptr,
    //&gyro
    nullptr
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
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();

    pros::Task screenTask([&]() {
        lemlib::Pose pose(0, 0, 0);
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            pros::delay(50);
        }
    });
}

void pollController()
{
    if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
        {
            ri.spin(ri.STANDARD_MV);
        }
        else if(driver.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {
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

void qualPB()
{
    chassis.setPose({-54, -54, 135});
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
    chassis.follow(path_txt, 15, 5000);
}

void matchPB()
{

}

void qualJ()
{

}

void matchJ()
{
	
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
        int r = driver.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        #if defined(ARCADE)
		    chassis.arcade(l, r);
	    #elif defined(TANK)
		    chassis.tank(l, r);
	    #endif

        pros::delay(10);
    }
}