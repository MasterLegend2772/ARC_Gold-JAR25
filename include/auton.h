#pragma once
#include "JAR-Template/drive.h"
#include "robot-config.h"

class Drive;

extern Drive chassis;

void defaultConstants();

void driveTest();
void turnTest();
void swingTest();
void fullTest();
void odomTest();
void tankOdomTest();
void holonomicOdomTest();


// ARC Gold Specific Functions && Variables
extern bool armUp;
extern bool isInAuton;
extern bool isSlotFull();

extern void toggleLift();
extern void toggleExtendo();
extern void outTake();
extern void moveSlot();
extern void unloadAll();


// Robot SETUP Functions
extern void setup15();
extern void setup24();

// Autonomous Miscellaneous Functions
extern void autonMatchload();

// Autonomous MATCH Route Functions
extern void autonMatch15();
extern void autonMatch24();

// Autonomous SKILLS Route Functions
extern void autonSkills15();
extern void autonSkills24();