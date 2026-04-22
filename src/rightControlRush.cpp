#include "main.h"
#include "auto.hpp"
#include "globals.hpp"
void rightControlRush()
{
// chassis.moveTo(0, 0, 5000);
// chassis.moveTo(0, 32.01, 5000);
// chassis.moveTo(17.969, 32.01, 5000);
// chassis.moveTo(-20.871, 33.01, 5000);
// chassis.moveTo(-20.936, 42.679, 5000);
// chassis.moveTo(-36.015, 42.679, 5000);

//setup
  chassis.setPose(0,0,0); //X and Y might be changed btw,
  wings.set_value(true);
  redirect.set_value(false);
  firstStageMotor.move(127);
  secondStageMotor.move(-127);
  //Right Inbetween
  chassis.moveToPoint(0,35.51,950,{},false);
  //Right Match Loader
  scraper.set_value(true);
  chassis.turnToPoint(17.969,35.51,570,{},false);
  chassis.moveToPoint(19.969,35.51,695,{.maxSpeed=40},false);
  //Right Long Goal
  chassis.turnToPoint(-20.871,36.01,250,{.forwards = false}, false);
  chassis.moveToPoint(-20.871,36.01, 1000, {.forwards = false, .minSpeed = 60}, true);
  pros::delay(650);
  wings.set_value(false);
  scraper.set_value(false);
  pros::delay(800);
  //wing
  chassis.moveToPoint(-8, 36.01, 700,{.minSpeed=70}, false);
  chassis.turnToPoint(-16.936, 45.679,350,{.forwards=false}, false);
  chassis.moveToPoint(-16.936, 45.679,620,{.forwards=false,.minSpeed=70}, false);
  chassis.turnToPoint(-36.015, 45.679, 350,{.forwards=false}, false);
  chassis.moveToPoint(-41.015, 45.679, 1200,{.forwards=false,.maxSpeed=110, .minSpeed=70}, false);
  leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  leftMotors.brake();
  rightMotors.brake();
}