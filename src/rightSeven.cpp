#include "main.h"
#include "auto.hpp"
#include "globals.hpp"
void rightSeven()
{
// chassis.moveTo(0, 0, 5000);
// chassis.moveTo(-27.052, 10.922, 5000);
// chassis.moveTo(-7.393, 30.581, 5000);
// chassis.moveTo(13.274, 34.614, 5000);
// chassis.moveTo(-20.835, 35.286, 5000);
// chassis.moveTo(-11.258, 35.286, 5000);
// chassis.moveTo(-19.659, 43.351, 5000);
// chassis.moveTo(-40.158, 42.679, 5000);



//setup
chassis.setPose(0,0,270); //X and Y might be changed btw,
wings.set_value(true);
redirect.set_value(false);
firstStageMotor.move(127);
secondStageMotor.move(-127);
//All Poses;
lemlib::Pose RIGHT_CLUSTER = lemlib::Pose(-23.644,9.01);
lemlib::Pose RIGHT_INBETWEEN = lemlib::Pose(0,30);
lemlib::Pose MATCHLOADER = lemlib::Pose(17.979, 30);
lemlib::Pose LONG_GOAL = lemlib::Pose(-21.5,30);
//lemlib::Pose LONG_GOAL_MIDWAY = chassis.getPose().lerp(LONG_GOAL,0.75);
lemlib::Pose MOVE_BACK = lemlib::Pose(-10,32.5);
lemlib::Pose FIRST_WING_MOVEMENT = lemlib::Pose(-14,39.5);
lemlib::Pose WING_MOVEMENT = lemlib::Pose(-41,39.5);
//First Cluster
chassis.moveToPoint(RIGHT_CLUSTER.x,RIGHT_CLUSTER.y,600,{.minSpeed=100,.earlyExitRange=1},true);
pros::delay(400);
scraper.set_value(true);
pros::delay(200);
//Inbetween
chassis.turnToPoint(RIGHT_INBETWEEN.x,RIGHT_INBETWEEN.y,500,{.forwards=false,.minSpeed=45,.earlyExitRange=5},false);
chassis.moveToPoint(RIGHT_INBETWEEN.x,RIGHT_INBETWEEN.y,1100,{.forwards=false},false);
//Matchloader
chassis.turnToPoint(MATCHLOADER.x,MATCHLOADER.y,650,{},false);
chassis.moveToPoint(MATCHLOADER.x,MATCHLOADER.y,600,{},false);
//Long Goal
chassis.turnToPoint(LONG_GOAL.x,LONG_GOAL.y,250,{.forwards = false}, false);
//chassis.moveToPoint(LONG_GOAL_MIDWAY.x,LONG_GOAL_MIDWAY.y,700,{.forwards=false,.minSpeed=127,.earlyExitRange=3});
chassis.moveToPoint(LONG_GOAL.x,LONG_GOAL.y,550,{.forwards=false,.minSpeed=127},false);
chassis.moveToPoint(LONG_GOAL.x,LONG_GOAL.y, 650, {.forwards=false,.maxSpeed=60}, true);
wings.set_value(false);
scraper.set_value(false);
pros::delay(1500);
//MoveBack
chassis.moveToPoint(MOVE_BACK.x,MOVE_BACK.y,400,{.minSpeed=127,.earlyExitRange=1},false);
//wing with attempt at motion chaining
wings.set_value(false);
chassis.turnToPoint(FIRST_WING_MOVEMENT.x,FIRST_WING_MOVEMENT.y,1000,{.forwards=false,.minSpeed=80,.earlyExitRange=10},false);
chassis.moveToPoint(FIRST_WING_MOVEMENT.x,FIRST_WING_MOVEMENT.y,1000,{.forwards=false},false);
chassis.turnToPoint(WING_MOVEMENT.x,WING_MOVEMENT.y,500,{.forwards=false},false);
chassis.moveToPoint(WING_MOVEMENT.x,WING_MOVEMENT.y,1000,{.forwards=false,.maxSpeed=860},false);
}
