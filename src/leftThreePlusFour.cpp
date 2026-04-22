#include "main.h"
#include "auto.hpp"
#include "globals.hpp"
void leftThreePlusFour(){
  /*chassis.moveTo(0, 0, 5000);
chassis.moveTo(0, 31.665, 5000);
chassis.moveTo(-17.875, 31.665, 5000);
chassis.moveTo(23.006, 32.165, 5000);
chassis.moveTo(24.231, 7.787, 5000);
chassis.moveTo(34.853, -2.835, 5000);
chassis.moveTo(10.859, 23.159, 5000);
chassis.moveTo(33.853, 23.159, 5000);



*/
  //setup
  chassis.setPose(0,0,0); //X and Y might be changed btw,
  wings.set_value(true);
  redirect.set_value(false);
  firstStageMotor.move(127);
  secondStageMotor.move(-127);
  //InBetween
  chassis.moveToPoint(0,35.51, 950, {}, false);
  //MatchLoader
  scraper.set_value(true);
  chassis.turnToPoint(-17.875,35.51, 440, {}, false);
  chassis.moveToPoint(-17.875, 35.51, 850, {.maxSpeed=50}, false);
  //Long Goal
  // chassis.turnToPoint(23.006,35.51, 400, {.forwards=false}, false);
  chassis.moveToPoint(23.006, 36.01, 1200, {.forwards = false}, true);
  pros::delay(700);
  wings.set_value(false);
  scraper.set_value(false);
  pros::delay(850);
  //Cluster
  chassis.turnToPoint(22.231,7.787,1000, {.minSpeed=100}, false);
  wings.set_value(true);
  chassis.moveToPoint(22.231,7.787,900,{.minSpeed=70,.earlyExitRange=5}, true);
  
  scraper.set_value(true);
  chassis.moveToPoint(22.231,7.787, 150,{.maxSpeed=70}, false);
 
  scraper.set_value(true);
   pros::delay(300);
  //MidGoal
  chassis.turnToPoint(34.853, -2.834, 600, {.forwards=false}, false);
  chassis.moveToPose(34.853,-2.83,315,750,{.forwards = false, .minSpeed=60},true);
  pros::delay(650);
  redirect.set_value(true);
  pros::delay(200);
  firstStageMotor.move(127);
  secondStageMotor.move(-82-14);
  pros::delay(650+150);//increased bc smth isnt working with mid
  scraper.set_value(false);
  //Wing
  chassis.turnToPoint(10.859,27.05-.5, 350, {}, false);
  chassis.moveToPoint(10.859, 27.05-.5, 1000, {.minSpeed = 49, .earlyExitRange = 0.5}, false);
  chassis.turnToPoint(35.859,27.05-.5,500,{.forwards=false},false);
  wings.set_value(false);
  chassis.moveToPoint(35.859,27.05-.5, 1500, {.forwards=false,.minSpeed=127, .earlyExitRange=20});
  chassis.moveToPoint(35.859,27.05-.5, 1500, {.forwards=false,.maxSpeed=60});

  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
}