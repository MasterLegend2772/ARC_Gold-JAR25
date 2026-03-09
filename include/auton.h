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

extern void toggleParkDescore();
extern void toggleLift();
extern void toggleExtendo();
extern void outTake();
extern void moveSlot();
extern void unloadAll();

// Setup for Robots
extern void setup15();
extern void setup24();

// Autonomous Skills Route Functions
extern void autonSkills15();
extern void autonSkills24();
extern void skillsRoute2();