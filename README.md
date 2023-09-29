# over-under-23-24

## Goals
- Real-time position tracking using inertial sensors (odometry)
- Auto-aim for turret
- Create multiple auton paths for each robot

## Resources
[PROS C++ Documentation](https://pros.cs.purdue.edu/v5/api/cpp/index.html)

[PiLons Odometry Paper](http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf)

[Previous Year Code](https://github.com/emcode25/TEK23-Spin-Up/tree/main/TEK23)

## Directory
Tom and Jerry are separate robot code projects. Tom is where we will do most of our code testing and implementation since it has the most complex systems.

## Structure
Each subsystem should be its own class (e.g. Drivetrain, Intake, Catapult, etc.).
### Execution
The logic to interact with these subsystems should be handled in opcontrol. Each autonomous routine should be written as a separate function.
