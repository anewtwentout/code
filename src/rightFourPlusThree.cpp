#include "main.h"
#include "auto.hpp"
#include "globals.hpp"
void rightFourPlusThree()
{
// chassis.moveTo(0, 0, 5000);
// chassis.moveTo(0, 32.01, 5000);
// chassis.moveTo(17.969, 32.01, 5000);
// chassis.moveTo(-20.871, 33.01, 5000);
// chassis.moveTo(-23.644, 9.01, 5000);
// chassis.moveTo(-36.612, -4.33, 5000);

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
  chassis.turnToPoint(17.969,35.51,400,{},false);
  chassis.moveToPoint(19.969,35.51,695,{.maxSpeed=40},false);
  //Right Long Goal
  chassis.turnToPoint(-20.871,36.01,250,{.forwards = false}, false);
  chassis.moveToPoint(-20.871,36.01, 1000, {.forwards = false, .minSpeed = 60}, true);
  pros::delay(650);
  wings.set_value(false);
  scraper.set_value(false);
  pros::delay(800);
  //Cluster
  chassis.turnToPoint(-26.533,5.112,300, {.minSpeed=127}, false);
  wings.set_value(true);
  secondStageMotor.move(0);
  chassis.moveToPoint(-26.533, 5.112, 500, {.maxSpeed=55}, false);
  //scraper.set_value(true);
  //pros::delay(300);
  //MidGoal
  chassis.turnToPoint(-38+.5, .3-.5-1.5, 500, {}, false);
  scraper.set_value(false);
  chassis.moveToPoint(-38+.5 ,.3-.5-1.5, 300, {.maxSpeed=100}, true);
  scraper.set_value(true);
  pros::delay(600);
  scraper.set_value(false);
  pros::delay(100+1000);
  chassis.moveToPoint(-38+.5 ,.3-.5-1.5, 1200, {}, false);
  intakeLift.set_value(true);
  firstStageMotor.move(-80);
  secondStageMotor.move(80);
  scraper.set_value(false);
  pros::delay(1600);
  //Wing
  chassis.moveToPose(-10.629, 47.75,90,1450, {.forwards = false, .minSpeed=100}, false);
  chassis.turnToPoint(-44.967,47.75,500,{.forwards=false},false);
  wings.set_value(false);
  //chassis.moveToPoint(-28.967, 47.75, 1000, {.forwards=false, .minSpeed = 100, .earlyExitRange=5}, false);
  chassis.moveToPoint(-44.967, 47.75-.25, 1000, {.forwards=false, .maxSpeed = 127}, false);
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  intakeLift.set_value(false);
}