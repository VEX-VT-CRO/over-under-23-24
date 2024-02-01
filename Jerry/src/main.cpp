//JERRY
//Turret robot capable of auto-aim

#include "main.h"
#include "subsystems/tankRobot.hpp"
#include <cstdlib>
#include "lemlib/api.hpp"

pros::Motor leftFront(11, true);
pros::Motor leftMiddle(12, true);
pros::Motor leftBack(13, true);

pros::Motor rightFront(19, false);
pros::Motor rightMiddle(18, false);
pros::Motor rightBack(20, false);

pros::Motor leftside[] = {leftFront, leftMiddle, leftBack};
pros::Motor rightside[] = {rightFront, rightMiddle, rightBack};

//16 - Spool

pros::Motor intake1(17);
pros::MotorGroup riGroup({intake1});
RollerIntake ri(riGroup);

pros::Motor turretMotor1(2);
pros::Motor turretMotor2(15);
pros::IMU turretGyro(5);
Turret* turret;

//3 - rotation sensor
pros::ADIDigitalIn catapult_charged('F');
pros::Distance distance_sensor(14);
pros::Motor catapultMotor(4);
Catapult* catapult;

pros::ADIDigitalOut indexerSolenoid('E');

TankDrivetrain drivetrain(leftside, rightside, 3);

//pros::Vision vision_sensor(10);
VisionSensor* vis;

TeamColor team = TeamColor::Blue;


TankRobot* robot;

//LEMLIB (https://lemlib.github.io/LemLib/md_docs_tutorials_2_setting_up_the_chassis->html)
pros::MotorGroup leftSideGroup({leftFront, leftMiddle, leftBack});
pros::MotorGroup rightSideGroup({rightFront, rightMiddle, rightBack});

lemlib::Drivetrain_t LLDrivetrain
{
	&leftSideGroup,
	&rightSideGroup,
	10.875, //Track width (space between groups in inches)
	3.215, //Wheel diameter
	266, //Wheel rpm
};

pros::IMU gyro(1);

pros::ADIEncoder verticalEncoder('A', 'B', true);
pros::Rotation horizontalRotation(7);

//Parameters: ADIEncoder, wheel diameter, distance from center, gear ratio
lemlib::TrackingWheel verticalWheel(&verticalEncoder, 2.9, 0.125, 1);
lemlib::TrackingWheel horizontalWheel(&horizontalRotation, 2.9, 5.5, 1);

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
	30, // kP
    140, // kD
    0.25, // smallErrorRange
    250, // smallErrorTimeout
    1, // largeErrorRange
    1000, // largeErrorTimeout
    10 // slew rate
};

lemlib::ChassisController_t turnController {
    5, // kP
    20, // kD
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

void goTo(float x, float y, int timeout, float maxSpeed = 127.0f, bool reversed = false, bool log = false)
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
lemlib::Pose target = lemlib::Pose(-48,0,0);

void autoaim(){
	turret->updatePosition(chassis->getPose(), target);
}

void initialize() {
	chassis = new lemlib::Chassis(LLDrivetrain, driveController, turnController, sensors);
	chassis->calibrate();
	chassis->setPose(0,0,0);
	turret = new Turret(turretMotor1, turretMotor2, turretGyro);
	//vis = new VisionSensor(vision_sensor);
	catapult = new Catapult(&catapultMotor, &catapult_charged, &distance_sensor);

	robot = new TankRobot(drivetrain, ri, nullptr, turret, nullptr, catapult, team);
	pros::lcd::initialize();

	
	pros::Task screenTask1(screen);
	// pros::Task screenTask2(autoaim);
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
	chassis->setPose(-36, -60, 90);
	//robot->goTo({36, 30.5}, 15000); //
	//chassis->moveTo(36, -36, 5000, 50);
	goTo(36, -60, 5000);
	//robot->goTo({47, 59.75}, 15000); //First triball
	goTo(48, -48, 5000);
	//robot->goTo({63.5, 59.75}, 15000); //Second triball
	goTo(53, -53, 15000);
	//robot->goTo({28.5, 14}, 15000); //Left of bar
	goTo(48, -48, 15000);
	//robot->goTo({99.5, 14}, 15000); //Right of bar
	goTo(60, -36, 15000);
	//robot->goTo({108, 29}, 15000); //Get ready for the turn
	//goTo(108, 29, 15000);
	//robot->goTo({94, 47}, 15000); //About to go to third triball
	//goTo(94, 47, 15000);
	//robot->goTo({77.5, 47}, 15000); //Third triball
	//goTo(77.5, 47, 15000);
	//robot->goTo({110.5, 58.75}, 15000); //Push it in
	//goTo(110.5, 58.75, 15000);
	//robot->goTo({75.5, 23.5}, 15000); //Ram the climb post
	//goTo(75.5, 23.5, 15000);

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


	//chassis->setPose(0, 0, 0);
	//chassis->moveTo(0, 10, 3000);
	//chassis->turnTo(5, 8.66, 5000);
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
	autonomous();
	while (true) {
		robot->pollController(false);
		
		pros::delay(20);
	}
}