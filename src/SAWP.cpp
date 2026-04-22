#include "main.h"
#include "auto.hpp"
#include "globals.hpp"
void SAWP(){
  //setup
  chassis.setPose(0,0,0); //X and Y might be changed btw,
  wings.set_value(true);
  redirect.set_value(false);
  firstStageMotor.move(127);
  secondStageMotor.move(-127);
  //Right Inbetween
  chassis.moveToPoint(0,34.75,950,{},false);
  //Right Match Loader
  scraper.set_value(true);
  chassis.turnToPoint(17.969,34.75,570,{},false);
  chassis.moveToPoint(19.969,34.75,685,{.maxSpeed=40},false);
  //Right Long Goal
  chassis.turnToPoint(-20.871-.5,35.75,250,{.forwards = false}, false);
  chassis.moveToPoint(-20.871-.5,35.75, 400, {.forwards = false, .minSpeed = 60}, false);
  chassis.moveToPoint(-20.871-.5,35.75,500,{.forwards=false},false);
  wings.set_value(false);
  scraper.set_value(false);
  pros::delay(950);
  //First Cluster
  chassis.turnToPoint(-23.644,9.01,800, {.minSpeed=70}, false);
  wings.set_value(true);
  chassis.moveToPoint(-23.644, 9.01, 700, {.maxSpeed=40}, false);
  // pros::delay(300);
  //scraper.set_value(true);
  //pros::delay(700);}
//   //Second Cluster
  chassis.turnToPoint(-23.644, -38.99, 150, {}, false);
  chassis.moveToPoint(-23.644, -35.99, 700,  {.minSpeed=127}, false);
  chassis.moveToPoint(-23.644, -42.99-2, 350, {.maxSpeed=40}, false);
  pros::delay(280);
 // scraper.set_value(true);
 // scraper.set_value(true);
  pros::delay(0520);
  //Left Inbetween
  chassis.turnToPoint(0,-63, 350, {}, false);
  chassis.moveToPoint(0,-63, 850, {}, false);
  //Left Long Goal
  chassis.turnToPoint(-20.905, -63, 450, {.forwards = false}, false);
  chassis.moveToPoint(-20.905, -63, 450, {.forwards = false}, false);
  chassis.moveToPoint(-20.905, -63, 800, {.forwards = false,.maxSpeed=60}, true);
  scraper.set_value(true);
  wings.set_value(false);
  pros::delay(1200);
  //Left MatchLoader
  chassis.turnToPoint(18.032+1, -62.5+2.75, 250, {}, false);
  chassis.moveToPoint(18.032+1, -62.5+2.75, 400, {.minSpeed = 127, .earlyExitRange = 3}, false);
  wings.set_value(true);
  chassis.moveToPoint(18.032+1,-62.5+2.75, 950, {.maxSpeed = 45}, false);
  //Before Mid Goal
  // chassis.turnToPoint(-23.644,-38.99, 450, {.forwards=false}, false);
  // chassis.moveToPoint(-23.644,-38.99,1000, {.forwards = false}, false);
  // scraper.set_value(false);
  // pros::delay(10000);
  //Mid Goal
  //chassis.turnToPoint(-36.644, -23.99, 500, {.forwards = false}, false);
 
  chassis.moveToPoint(6, -62.99,  300,{.forwards=false}, false);
  chassis.turnToPoint(-36.644+3+0.5+1+1+1.25-2.5-.5+.25-1.5-.25, -23.99-3+1+1+1.25-2.5+3.5-.25+1.5, 100, {.forwards = false}, false);
  chassis.moveToPose(-36.644+3+0.5+1+1+1.25-2.5-.5+.25-1.5-.25, -23.99-3+1+1+1.25-2.5+3.5-.25+1.5, 135, 1680-200, {.forwards = false, .minSpeed = 60}, false);
  chassis.moveToPose(-36.644+3+0.5+1+1+1.25-2.5-.5+.25-1.5-.25, -23.99-3+1+1+1.25-2.5+3.5-.25+1.5,135,500,{},true);
  pros::delay(350);
  redirect.set_value(true);
  firstStageMotor.move(127);
  secondStageMotor.move(-45);
}