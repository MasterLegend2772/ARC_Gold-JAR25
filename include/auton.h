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
extern void outTake();
extern void moveSlot();
extern void unloadAll();
extern void autonSkills();
extern void skillsRoute2();