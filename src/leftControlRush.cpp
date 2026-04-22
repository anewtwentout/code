#include "main.h"
#include "auto.hpp"
#include "globals.hpp"
void leftControlRush(){
  chassis.setPose(0,0,0); //X and Y might be changed btw,
  wings.set_value(true);
  redirect.set_value(false);
  firstStageMotor.move(127);
  secondStageMotor.move(-127);
   //InBetween
  chassis.moveToPoint(0,35.51, 950, {}, false);
  //MatchLoader
  scraper.set_value(true);
  chassis.turnToPoint(-17.875,35.51, 400, {}, false);
  chassis.moveToPoint(-17.875, 35.51, 850, {.maxSpeed=50}, false);
  //Long Goal
  // chassis.turnToPoint(23.006,35.51, 400, {.forwards=false}, false);
  chassis.moveToPoint(23.006, 35.51, 1200, {.forwards = false}, true);
  pros::delay(700);
  wings.set_value(false);
  scraper.set_value(false);
  pros::delay(850);
  //wing
  chassis.moveToPoint(8, 36.01, 700,{.minSpeed=70}, false);
  chassis.turnToPoint(16.936, 26.341,350,{.forwards=false}, false);
  chassis.moveToPoint(16.936, 26.341,620,{.forwards=false,.minSpeed=70}, false);
  chassis.turnToPoint(36.015, 26.341, 350,{.forwards=false}, false);
  chassis.moveToPoint(41.015, 26.341, 1200,{.forwards=false,.maxSpeed=110, .minSpeed=70}, false);
  leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  leftMotors.brake();
  rightMotors.brake();
}