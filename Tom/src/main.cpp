//TOM
//Senior robot

#include "main.h"
#include "subsystems/tankRobot.hpp"
#include <cstdlib>
#include "lemlib/api.hpp"

pros::Motor leftFront(14, true);
pros::Motor leftMiddle(15, true);
pros::Motor leftBack(16, true);

pros::Motor rightFront(17, false);
pros::Motor rightMiddle(18, false);
pros::Motor rightBack(19, false);

pros::Motor leftside[] = {leftFront, leftMiddle, leftBack};
pros::Motor rightside[] = {rightFront, rightMiddle, rightBack};


pros::Motor intake1(11, false);
pros::Motor intake2(12, true);
pros::MotorGroup riGroup({intake1, intake2});
RollerIntake ri(riGroup);


pros::ADIDigitalIn catapult_charged('H');
// pros::Distance distance_sensor(19);
pros::Motor catapultMotor1(9,true);
pros::Motor catapultMotor2(10,false);
// pros::Motor catapult_motors[] = {catapultMotor1, catapultMotor2};
Catapult* catapult;

pros::ADIDigitalOut open_intake_sol('G');
pros::ADIDigitalOut solenoid('F');
Indexer* indexer;

TankDrivetrain drivetrain(leftside, rightside, 3);

TeamColor team = TeamColor::Blue;


TankRobot* robot;

//LEMLIB (https://lemlib.github.io/LemLib/md_docs_tutorials_2_setting_up_the_chassis->html)
pros::MotorGroup leftSideGroup({leftFront, leftMiddle, leftBack});
pros::MotorGroup rightSideGroup({rightFront, rightMiddle, rightBack});

lemlib::Drivetrain_t LLDrivetrain
{
	&leftSideGroup,
	&rightSideGroup,
	12, //Track width (space between groups in inches)
	4.14, //Wheel diameter
	257, //Wheel rpm
};

pros::IMU gyro(20);

pros::ADIEncoder verticalEncoder('D', 'E');
pros::ADIEncoder horizontalEncoder('B', 'C');

//Parameters: ADIEncoder, wheel diameter, distance from center, gear ratio
lemlib::TrackingWheel verticalWheel(&verticalEncoder, 2.72, 0, 1);
lemlib::TrackingWheel horizontalWheel(&horizontalEncoder, 2.75, 6, 1);

lemlib::OdomSensors_t sensors
{
	&verticalWheel,
	nullptr,
	&horizontalWheel,
	nullptr,
	&gyro
};

lemlib::ChassisController_t driveController
{
	750, // kP
    0, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    2 // slew rate
};

lemlib::ChassisController_t turnController {
    4, // kP
    40, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    0 // slew rate
};

lemlib::Chassis* chassis;

//Combine turnTo and moveTo into a single function
void goTo(float x, float y, int timeout, float maxDriveSpeed, float maxTurnSpeed, bool reversed = false, bool log = false)
{
	chassis->turnTo(x, y, timeout, reversed, maxTurnSpeed, log);
	chassis->moveTo(x, y, timeout, maxDriveSpeed, log);
}

void goTo(float x, float y, int timeout, float maxSpeed = 50.0f, bool reversed = false, bool log = false)
{
	chassis->turnTo(x, y, timeout, reversed, maxSpeed, log);
	chassis->moveTo(x, y, timeout, maxSpeed, log);
}

void screen() {
    // loop forever
    while (true) {
        lemlib::Pose pose = chassis->getPose(); // get the current position of the robot
        pros::lcd::print(0, "x: %f", pose.x); // print the x position
        pros::lcd::print(1, "y: %f", pose.y); // print the y position
        pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
        pros::delay(10);
    }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	chassis = new lemlib::Chassis(LLDrivetrain, driveController, turnController, sensors);
	chassis->calibrate();
	pros::Task screenTask(screen); // create a task to print the position to the screen
	catapult = new Catapult(&catapultMotor1, &catapultMotor2, &catapult_charged, nullptr);
	indexer = new Indexer(solenoid, open_intake_sol);
	robot = new TankRobot(drivetrain, ri, indexer, nullptr, catapult, team);

	pros::lcd::initialize();
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

void charge()
{
	catapult->spin(-6000);
	pros::delay(2500);
	catapult->spin(0);
}

void half()
{
	catapult->spin(-6000);
	pros::delay(750);
	catapult->spin(0);
}

void shoot()
{
	catapult->spin(-6000);
	pros::delay(800);
	catapult->spin(0);
}

//To be used in a qualifying match with Jerry
void jerryQual()
{
	indexer->openIntake();
	chassis->setPose(-53.7, -53.7, -135);
	chassis->moveTo(-36, -36, 2000);
	chassis->turnTo(48, 0, 2000);
	shoot();
	half();
	//BACK TO LOAD
	ri.spin(ri.STANDARD_MV);
	chassis->moveTo(-50, -50, 2000);
	ri.spin(0);
	goTo(-60, -36, 5000);
	chassis->turnTo(-60, -30, 2000, true);
	ri.spin(-ri.STANDARD_MV);
	pros::delay(500);
	chassis->turnTo(-60, -30, 2000, true);
	ri.spin(0);
	chassis->moveTo(-60, -30, 2000);
	chassis->moveTo(-60, -36, 2000);
	goTo(-36, -36, 2000);
	charge();
	ri.spin(ri.STANDARD_MV);
	goTo(-24, -12, 2000);
	chassis->turnTo(48, -12, 2000, false);
	shoot();
	charge();
	goTo(-9, -12, 2000);
	chassis->turnTo(48, -12, 2000, true);
	shoot();
	goTo(-12, -36, 2000);
	goTo(0, -48, 2000);
}

void elimAuto()
{
	indexer->openIntake();
	chassis->setPose(-53.7, -53.7, -135);
	chassis->moveTo(-36, -36, 2000);
	chassis->turnTo(48, 0, 2000, true);
	ri.spin(ri.STANDARD_MV);
	shoot();
	/*half();
	goTo(-53.7, -53.7, 2000);
	ri.spin(ri.STANDARD_MV);
	chassis->moveTo(-48, -48, 2000);
	ri.spin(0);
	goTo(-60, -36, 2000);
	chassis->turnTo(-60, -30, 2000);
	ri.spin(-ri.STANDARD_MV);
	pros::delay(1000);
	chassis->moveTo(-60, -40, 2000);
	chassis->turnTo(-60, -60, 2000, true);
	ri.spin(0);
	chassis->moveTo(-60, -30, 2000);
	chassis->moveTo(-60, -36, 2000);
	goTo(-48, -48, 2000);
	goTo(-53.7, -53.7, 2000);
	catapult->spin(-6000);
	pros::delay(1850);
	catapult->spin(0);
	ri.spin(ri.STANDARD_MV);

	pros::delay(750);
	chassis->moveTo(-36, -36, 2000);
	chassis->turnTo(48, 0, 2000, true);
	shoot();
	goTo(-53.7, -53.7, 2000);
	
	pros::delay(750);
	chassis->moveTo(-36, -36, 2000);
	chassis->turnTo(48, 0, 2000, true);
	shoot();
	goTo(-53.7, -53.7, 2000);

	pros::delay(750);
	chassis->moveTo(-36, -36, 5000);
	chassis->turnTo(48, 0, 5000, true);
	shoot();
	goTo(-53.7, -53.7, 5000);

	chassis->moveTo(-36, -36, 5000);
	chassis->turnTo(48, 0, 5000, true);
	shoot();
	goTo(-53.7, -53.7, 5000);

	chassis->moveTo(-36, -36, 5000);
	chassis->turnTo(48, 0, 5000, true);
	shoot();
	goTo(-53.7, -53.7, 5000);*/
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
	elimAuto();
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
	robot->start();
	while (true) {
		robot->pollController(false);
		if (catapult->charge_state) {
            catapult->charge();
        }
		pros::delay(10);
	}
}