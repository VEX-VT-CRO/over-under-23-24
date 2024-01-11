#include "main.h"
#include "subsystems/tankRobot.hpp"
#include <cstdlib>
#include "lemlib/api.hpp"

pros::Motor leftFront(1, true);
pros::Motor leftMiddle(2, true);
pros::Motor leftBack(3, true);

pros::Motor rightFront(11, false);
pros::Motor rightMiddle(12, false);
pros::Motor rightBack(13, false);

pros::Motor leftside[] = {leftFront, leftMiddle, leftBack};
pros::Motor rightside[] = {rightFront, rightMiddle, rightBack};


pros::Motor intake(7);
RollerIntake ri(intake);

pros::Motor turretMotor1(8);
pros::Motor turretMotor2(9);
pros::IMU turretGyro(17);
Turret* turret;

pros::ADIDigitalIn catapult_charged(18);
pros::Distance distance_sensor(19);
pros::Motor catapultMotor(15);
Catapult* catapult;

pros::ADIDigitalOut indexerSolenoid('E');
Indexer i(indexerSolenoid);

TankDrivetrain drivetrain(leftside, rightside, 3);

pros::Vision vision_sensor(10);
VisionSensor* vis;

TeamColor team = TeamColor::Blue;


TankRobot* robot;

//LEMLIB (https://lemlib.github.io/LemLib/md_docs_tutorials_2_setting_up_the_chassis.html)
pros::MotorGroup leftSideGroup({leftFront, leftMiddle, leftBack});
pros::MotorGroup rightSideGroup({rightFront, rightMiddle, rightBack});

lemlib::Drivetrain_t LLDrivetrain
{
	&leftSideGroup,
	&rightSideGroup,
	10, //Track width (space between groups in inches)
	3.25, //Wheel diameter
	360, //Wheel rpm
};

pros::IMU gyro(18);

pros::ADIEncoder verticalEncoder('C', 'D');
pros::ADIEncoder horizontalEncoder('A', 'B');

//Parameters: ADIEncoder, wheel diameter, distance from center, gear ratio
lemlib::TrackingWheel verticalWheel(&verticalEncoder, 2.75, 4.3, 1);
lemlib::TrackingWheel horizontalWheel(&horizontalEncoder, 2.75, 4.3, 1);

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
	8, // kP
    30, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    5 // slew rate
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

lemlib::Chassis chassis(LLDrivetrain, driveController, turnController, sensors);

//Combine turnTo and moveTo into a single function
void goTo(float x, float y, int timeout, float maxDriveSpeed, float maxTurnSpeed, bool reversed = false, bool log = false)
{
	chassis.turnTo(x, y, timeout, reversed, maxTurnSpeed, log);
	chassis.moveTo(x, y, timeout, maxDriveSpeed, log);
}

void goTo(float x, float y, int timeout, float maxSpeed = 127.0f, bool reversed = false, bool log = false)
{
	chassis.turnTo(x, y, timeout, reversed, maxSpeed, log);
	chassis.moveTo(x, y, timeout, maxSpeed, log);
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	turret = new Turret(turretMotor1, turretMotor2, turretGyro, {0, 0, 0});
	catapult = new Catapult(&catapultMotor, &catapult_charged, &distance_sensor);
	vis = new VisionSensor(vision_sensor);

	robot = new TankRobot(drivetrain, ri, i, turret, vis, catapult, team);

	pros::lcd::initialize();

	chassis.calibrate();
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
	//TEST AUTON
	//odom->setPosition({16, 30.5}); //START
	//odom->setAngle(0);
	chassis.setPose(16, 30.5, 0);
	//robot->goTo({36, 30.5}, 15000); //
	goTo(36, 30.5, 15000);
	//robot->goTo({47, 59.75}, 15000); //First triball
	goTo(47, 59.75, 15000);
	//robot->goTo({63.5, 59.75}, 15000); //Second triball
	goTo(63.5, 59.75, 15000);
	//robot->goTo({28.5, 14}, 15000); //Left of bar
	goTo(28.5, 14, 15000);
	//robot->goTo({99.5, 14}, 15000); //Right of bar
	goTo(99.5, 14, 15000);
	//robot->goTo({108, 29}, 15000); //Get ready for the turn
	goTo(108, 29, 15000);
	//robot->goTo({94, 47}, 15000); //About to go to third triball
	goTo(94, 47, 15000);
	//robot->goTo({77.5, 47}, 15000); //Third triball
	goTo(77.5, 47, 15000);
	//robot->goTo({110.5, 58.75}, 15000); //Push it in
	goTo(110.5, 58.75, 15000);
	//robot->goTo({75.5, 23.5}, 15000); //Ram the climb post
	goTo(75.5, 23.5, 15000);

	//TESTING PID
	/*while(odom->getPosition().y < 70.5)
	{
		leftFront.move_voltage(2250);
		leftMiddle.move_voltage(2250);
		leftBack.move_voltage(2250);
		rightFront.move_voltage(3000);
		rightMiddle.move_voltage(3000);
		rightBack.move_voltage(3000);
		
		pros::delay(10);
		odom->update();
	}

	drivetrain.drive(0);

	pros::delay(1500);

	while(odom->getPosition().y > 0.0)
	{
		leftFront.move_voltage(-2700);
		leftMiddle.move_voltage(-2700);
		leftBack.move_voltage(-2700);
		rightFront.move_voltage(-3000);
		rightMiddle.move_voltage(-3000);
		rightBack.move_voltage(-3000);
		
		pros::delay(10);
		odom->update();
	}*/
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

	while (true) {
		robot->pollController(false);
		
		pros::delay(20);
	}
}