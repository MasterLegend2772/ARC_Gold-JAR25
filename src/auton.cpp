#include "vex.h"

// Global Variables
int blockSlots = 0;

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
 * Function: setup15()
 * 
 * Purpose: Setup Call for 15 Inch Autonomous & Match
*******************************************************************/
void setup15() {
	// Returns Odometry Constants
	odom_constants();

	// Initialize Setup
	chassis.setCoordinates(75.592, 12.992, 255);
	extendo.set(true);
	revolver.setVelocity(100, percent);
	blockSlots = 0;
}


/******************************************************************
 * Function: setup24()
 * 
 * Purpose: Setup Call for 24 Inch Autonomous & Match
*******************************************************************/
void setup24() {
	// Returns Odometry Constants
	odom_constants();

	// Initialize Setup
	chassis.setCoordinates(56.442, 18.456, 270);
	revolver.setVelocity(100, percent);
	blockSlots = 0;
}


/******************************************************************
 * Function: autonMatchload()
 * 
 * Purpose: Moves Forward/Reverse to re-align with Matchload
*******************************************************************/
void autonMatchload() {
	while(!isSlotFull()) {
		chassis.driveDistance(-2);
		wait(0.125, seconds);
		chassis.driveDistance(2);
		wait(0.125, seconds);
	}

	// Rotate Slot
	intake.spin(forward, 0.00, volt);
	moveSlot();
	blockSlots++;
	intake.spin(forward, 12.00, volt);
}


/***********************************************************************
 * Function: autoScoreAll(bool)
 * 
 * Parameters:  [bool] matchCheck >> TRUE if non-skills match
 * 			    [int]  blueSlot   >> Value of Slot(s) containing
 * 									 opponent colored blocks
 * 
 * Purpose: Auto Score Slots with Blocks
 * 			Skips Blue Slot if given parameter
************************************************************************/
void autoScoreAll(bool matchCheck, int enemySlot = 0) { // No Blue Slot if no parameter || blueSlot == -1
	for (int i = 0; i < blockSlots; i++) {
		if (matchCheck && (i == (enemySlot - 1))) {
			moveSlot();
		} else {
			outTake();
			moveSlot();
		}
	}

	blockSlots = 0;
}


/******************************************************************
 * Function: autonMatch15()
 * 
 * Purpose: Autonomous MATCH Route for 15 Inch
*******************************************************************/
void autonMatch15() {
// Change setup position to Parking Zone EDGE
	// Update AUTON as necessary
// Setup
	setup15();
	revolver.spinToPosition(-360, degrees, true);

//Align to Matchloader1
	chassis.driveToPoint(103.150, 17.362);
	chassis.turnToAngle(178.5);
	matchLoader.set(true);
	intake.spin(forward, 12.00, volt);
	wait(0.5, seconds);

//Unload Matchloader
	chassis.driveToPoint(103.150, 6.25);
	wait(1.00, seconds);
	
// Auto Rotate Slot && Matchload Function
	// Remove While Loop if possible
		// MUST FIX INTAKE PROBLEM FIRST
	autonMatchload();
	wait(0.75, seconds);
	autonMatchload();
	wait(0.75, seconds);
	autonMatchload();
	wait(0.75, seconds);
	autonMatchload();


// Reverse to RED Side Blocks
	matchLoader.set(false);
	chassis.driveToPose(116.5, 23.211, 90);
	chassis.leftSwingToAngle(88.5);

// Intake Side Blocks
	intake.spin(forward, 12, volt);
	chassis.driveToPoint(120, 109);
	chassis.turnToAngle(180);

// Score in CLOSEST Long Goal
	toggleLift();
	autoScoreAll(true, 2);
	// Add Wait if needed
	toggleLift(); // Change as Needed


// Reverse to Driver Control Starting Position
	// Or block goal (depending on opponent auton)
	// Or use Descore Arm to block ??
}


/******************************************************************
 * Function: autonSkills24()
 * 
 * Purpose: Autonomous MATCH Route for 24 Inch
*******************************************************************/
void autonMatch24() {

}


/******************************************************************
 * Function: autonSkills15()
 * 
 * Purpose: Autonomous SKILLS Route for 15 Inch
*******************************************************************/
void autonSkills15() {  // 15 Inch Version
	// Setup
	setup15();

	//Align to Matchloader1
	chassis.driveToPoint(103.150, 17.362);
	chassis.turnToAngle(178.5);
	matchLoader.set(true);
	intake.spin(forward, 12.00, volt);
	wait(0.5, seconds);

	//Unload Matchloader
	chassis.driveToPoint(103.150, 6.25);
	wait(1.25, seconds);
	
	// Auto Rotate Slot
		// Conditional if Block Intake gets stuck
	while(!isSlotFull()) { // Tune & Test
		chassis.driveDistance(-2);
		wait(0.25, seconds);
		chassis.driveDistance(2);
		wait(0.25, seconds);
	}
		// Rotate Slot
	intake.spin(forward, 0.00, volt);
	moveSlot();
	intake.spin(forward, 12.00, volt);
	wait(0.25, seconds);
		// Conditional if Block Intake gets stuck
	while(!isSlotFull()) {
		chassis.driveDistance(-2);
		wait(0.25, seconds);
		chassis.driveDistance(2);
		wait(0.25, seconds);
	}

	//Travel Down the Side
	matchLoader.set(false);
	chassis.driveToPose(116.5, 35, 0);
	chassis.driveToPoint(116.5, 97);
	chassis.leftSwingToAngle(88.5);

	// Intake Side Blocks
	intake.spin(forward, 12, volt);
	chassis.driveToPoint(116, 109);


	// Continue as Needed
}


/******************************************************************
 * Function: autonSkills24()
 * 
 * Purpose: Autonomous SKILLS Route for 24 Inch
*******************************************************************/
void autonSkills24() {  // 24 Inch Version
	// Setup
	setup24();

	// Descore Red Park Zone
	chassis.driveToPoint(14.542, 18.456);

	// Descore Matchload
	chassis.turnToAngle(180);
	matchLoader.set(true);
	intake.spin(forward, 12.00, volt);
	wait(0.5, seconds);
	chassis.driveToPoint(14.542,  6.25);
	wait(1.25, seconds);
	while (!isSlotFull()) {
		// Add if Intake doesn't work still
	}
	intake.spin(forward, 0.00, volt);
	moveSlot();
	intake.spin(forward, 12.00, volt);
	wait(0.25, seconds);
	while (!isSlotFull()) {

	}
	moveSlot();

	// Intake Descored Park Zone Blocks
	chassis.driveToPoint(14.542,  12.25);
	chassis.turnToAngle(270);
	chassis.driveToPoint(2.5, 12.25);
	wait(0.5, seconds);
	moveSlot();
	wait(0.15, seconds);
	chassis.driveToPoint(0.0, 16.140);
	wait(0.25, seconds);
	moveSlot();

	// Continue as Needed
}