#include "vex.h"

/**
 * Resets the constants for auton movement.
 * Modify these to change the default behavior of functions like
 * driveDistance(). For explanations of the difference between
 * drive, heading, turning, and swinging, as well as the PID and
 * exit conditions, check the docs.
 */

void defaultConstants(){
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
  chassis.setDriveTrainConstants(10, 1.5, 0, 10, 0);
  chassis.setHeadingConstants(6, .4, 0, 1, 0);
  chassis.setTurnConstants(12, .4, .03, 3, 15);
  chassis.setSwingConstants(12, .3, .001, 2, 15);

  // Each exit condition set is in the form of (settle_error, settle_time, timeout).
  chassis.setDriveExitConditions(1.5, 300, 5000);
  chassis.setTurnExitConditions(1, 300, 3000);
  chassis.setSwingExitConditions(1, 300, 3000);
}

/**
 * Sets constants to be more effective for odom movements.
 * For functions like driveToPoint(), it's often better to have
 * a slower max_voltage and greater settle_error than you would otherwise.
 */

void odom_constants(){
  defaultConstants();
  chassis.headingMaxVoltage = 10;
  chassis.driveMaxVoltage = 8;
  chassis.driveSettleError = 3;
  chassis.boomerangLead = 0.5;
  chassis.driveMinVoltage = 1.45;
}

/**
 * The expected behavior is to return to the start position.
 */

void driveTest(){
  chassis.driveDistance(6);
  chassis.driveDistance(12);
  chassis.driveDistance(18);
  chassis.driveDistance(-36);
}

/**
 * The expected behavior is to return to the start angle, after making a complete turn.
 */

void turnTest(){
  chassis.turnToAngle(5);
  chassis.turnToAngle(30);
  chassis.turnToAngle(90);
  chassis.turnToAngle(225);
  chassis.turnToAngle(0);
}

/**
 * Should swing in a fun S shape.
 */

void swingTest(){
  chassis.leftSwingToAngle(90);
  chassis.rightSwingToAngle(0);
}

/**
 * A little of this, a little of that; it should end roughly where it started.
 */

void fullTest(){
  chassis.driveDistance(24);
  chassis.turnToAngle(-45);
  chassis.driveDistance(-36);
  chassis.rightSwingToAngle(-90);
  chassis.driveDistance(24);
  chassis.turnToAngle(0);
}

/**
 * Doesn't drive the robot, but just prints coordinates to the Brain screen 
 * so you can check if they are accurate to life. Push the robot around and
 * see if the coordinates increase like you'd expect.
 */

void odomTest(){
  chassis.setCoordinates(0, 0, 0);
  while(1){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5,20, "X: %f", chassis.getXPosition());
    Brain.Screen.printAt(5,40, "Y: %f", chassis.getYPosition());
    Brain.Screen.printAt(5,60, "Heading: %f", chassis.getAbsoluteHeading());
    Brain.Screen.printAt(5,80, "ForwardTracker: %f", chassis.getForwardTrackerPosition());
    Brain.Screen.printAt(5,100, "SidewaysTracker: %f", chassis.getSidewaysTrackerPosition());
    task::sleep(20);
  }
}

/**
 * Should end in the same place it began, but the second movement
 * will be curved while the first is straight.
 */

void tankOdomTest(){
  odom_constants();
  chassis.setCoordinates(0, 0, 0);
  chassis.turnToPoint(24, 24);
  chassis.driveToPoint(24,24);
  chassis.driveToPoint(-24,-24);
  chassis.turnToAngle(0);
}

/*
 * Drives in a square while making a full turn in the process. Should
 * end where it started.
 */

void holonomicOdomTest(){
  odom_constants();
  chassis.setCoordinates(0, 0, 0);
  chassis.holonomicDriveToPose(0, 18, 90);
  chassis.holonomicDriveToPose(18, 0, 180);
  chassis.holonomicDriveToPose(0, 18, 270);
  chassis.holonomicDriveToPose(0, 0, 0);
}




// ARC Gold Specific Functions && Variables

/******************************************************************
 * Function: autonSkills()
 * 
 * Purpose: Autonomous Skills Route
*******************************************************************/
void autonSkills() {  // 15 Inch Version
  // Returns Odometry Constants
  odom_constants();

  // Drive to Matchload
  chassis.setCoordinates(75.592, 12.992, 255);
  chassis.driveToPoint(103.583, 17.992);
  chassis.turnToAngle(178.5);
  wait(0.15, seconds);

  // Red Matchload (+3 Red || +3 Blue)
  matchLoader.set(true);
  // intake.spin(forward, 12.00, volt);
  chassis.driveDistance(8.84);
  wait(1.75, seconds);

  // Drive to get Sideblocks (+2 Blue)
  chassis.driveDistance(-4.95);
  wait(0.1, seconds);
  matchLoader.set(false);
  chassis.driveDistance(-6.00);
  chassis.turnToAngle(88.5);
  wait(0.15, seconds);
  chassis.driveDistance(13.46);
  wait(0.55, seconds);

  // Drive to Score in CLOSEST Long Goal
  chassis.driveDistance(-13.53);
  chassis.turnToAngle(-1.5);
  wait(0.15, seconds);
  chassis.driveDistance(13.55);
  // toggleLift();
  // outTake();
  // moveSlot();
  // outTake();
  // moveSlot();
  // outTake();
  // wait(0.1, seconds);
  // toggleLift();

  // Drive to Intake BLUE Side Blocks
  chassis.driveDistance(-7);
  chassis.leftSwingToAngle(-178.5);
  wait(0.15, seconds);
  chassis.driveToPoint(117.294, 98.575);
  chassis.leftSwingToAngle(88.5);
  wait(0.15, seconds);
  chassis.driveDistance(8.5);
  wait(0.55, seconds);

  // Blue Matchload
  chassis.driveDistance(-13.43);
  chassis.turnToAngle(-1.5);
  wait(0.15, seconds);
  matchLoader.set(true);
  chassis.driveDistance(10.55);
  wait(1.75, seconds);
  chassis.driveDistance(-10.95);
  matchLoader.set(false);
  chassis.turnToAngle(178.5);
  chassis.driveDistance(11.45);
  chassis.turnToAngle(178.5);
  wait(0.15, seconds);
  // toggleLift();
  // outTake();
  // moveSlot();
  // outTake();
  // moveSlot();
  // outTake();
  // wait(0.1, seconds);
  // toggleLift();

  // Drive to Park Zone && Park
  chassis.driveDistance(-7);
  chassis.leftSwingToAngle(1.5);
  wait(0.15, seconds);
  chassis.driveToPoint(84.488, 15.578);
  chassis.turnToAngle(91.5);
  chassis.driveDistance(-18.25);

  // End
}

void skillsRoute2()
{
  // Returns Odometry Constants
  odom_constants();

  // Drive to Matchload
  chassis.setCoordinates(75.592, 12.992, 255);
  toggleLift();
  extendo.set(true);
  wait(0.25, sec);
  toggleLift();

  //Align to Matchloader1
  chassis.driveToPoint(103.583, 17.992);
  chassis.turnToAngle(178.5);
  matchLoader.set(true);
  intake.spin(forward, 12.00, volt);
  wait(0.5, seconds);

  //Unload Matchloader
  //chassis.driveDistance(12);
  chassis.driveToPoint(103, 6.25);
  while(!isSlotFull())
  {
    chassis.turnToAngle(178);
    chassis.turnToAngle(182);
  }
  chassis.turnToAngle(180);
  intake.spin(forward, 0.00, volt);
  moveSlot();
  intake.spin(forward, 12.00, volt);
  while(!isSlotFull())
  {
    chassis.turnToAngle(172);
    chassis.turnToAngle(182);
  }
  chassis.turnToAngle(180);
  intake.spin(forward, 0.00, volt);
  moveSlot();

  //Travel Down the Side
  matchLoader.set(false);
  chassis.driveToPoint(116.5, 35);
  chassis.turnToAngle(180);
  chassis.driveToPoint(116.5, 97);
  chassis.driveToPoint(100, 109);
  chassis.turnToAngle(90);

  intake.spin(forward, 12, volt);
  chassis.driveToPoint(116, 109);
}