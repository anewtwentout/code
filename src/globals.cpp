#include "globals.hpp"
pros::Controller controller(pros::E_CONTROLLER_MASTER);
//pnematics
pros::adi::DigitalOut scraper ('D', false);
pros::adi::DigitalOut intakeLift ('A', false);
pros::adi::DigitalOut wings ('B', false);
pros::adi::DigitalOut redirect ('C', false);
pros::adi::DigitalOut midGoal_descore ('E', false);
// motor groups
pros::MotorGroup leftMotors({-11, -12, -13}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({14, 15, 16}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)
pros::Motor firstStageMotor(1, pros::MotorGearset::blue);
pros::Motor secondStageMotor(2, pros::MotorGearset::blue);
//Distance Sensors
pros::Distance frontDistance(8);
pros::Distance rightDistance(9);
pros::Distance leftDistance(10);
// Inertial Sensor on port 17
pros::Imu imu(17);
//Odom
pros::Rotation vertical_encoder(-18);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, -0.4);
/*
pros::Distance distance(1);
chassis.setPos(chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
*/
// tracking wheels
//horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
// pros::Rotation horizontalEnc(0);
// //vertical tracking wheel encoder. Rotation sensor, port 11, reversed
// pros::Rotation verticalEnc(-11);
//horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)

//vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
// lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              12, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller

lemlib::ControllerSettings linearController( 7, // proportional gain (kP)
                                            0, // integral gain (kI)
                                           40, // derivative gain (kD)
                                            0, // anti windup
                                            0, // small error range, in inches
                                            0, // small error range timeout, in milliseconds
                                            0, // large error range, in inches
                                            0, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller

lemlib::ControllerSettings angularController(1.9, // proportional gain (kP)
                                             0,// integral gain (kI)
                                            12, // derivative gain (kD)
                                             0, // anti windup
                                             0, // small error range, in degrees
                                             0, // small error range timeout, in milliseconds
                                             0, // large error range, in degrees
                                             0, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(5, // joystick deadband out of 127
                                     5, // minimum output where drivetrain will move out of 127
                                     1.012 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  5, // minimum output where drivetrain will move out of 127
                                  1.012 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);
