#pragma once
#include "main.h"
#include "lemlib/api.hpp"
extern pros::Controller controller;
//pnematics
extern pros::adi::DigitalOut scraper;
extern pros::adi::DigitalOut intakeLift;
extern pros::adi::DigitalOut wings;
extern pros::adi::DigitalOut redirect;
extern pros::adi::DigitalOut midGoal_descore;
// motor groups
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors; // right motor group - ports 6, 7, 9 (reversed)
extern pros::Motor firstStageMotor;
extern pros::Motor secondStageMotor;
//Distance Sensors
extern pros::Distance frontDistance;
extern pros::Distance rightDistance;
extern pros::Distance leftDistance;
// Inertial Sensor on port 17
extern pros::Imu imu;
//Odom
extern pros::Rotation vertical_encoder;
extern lemlib::TrackingWheel vertical_tracking_wheel;
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
extern lemlib::Drivetrain drivetrain;
// lateral motion controller

extern lemlib::ControllerSettings linearController;

// angular motion controller

extern lemlib::ControllerSettings angularController;

// sensors for odometry
extern lemlib::OdomSensors sensors;
// input curve for throttle input during driver control
extern lemlib::ExpoDriveCurve throttleCurve;

// input curve for steer input during driver control
extern lemlib::ExpoDriveCurve steerCurve;

// create the chassis
extern lemlib::Chassis chassis;
