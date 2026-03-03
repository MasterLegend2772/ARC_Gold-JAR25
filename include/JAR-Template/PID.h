#pragma once
#include "vex.h"

/**
 * General-use PID class for drivetrains. It includes both
 * control calculation and settling calculation. The default
 * update period is 10ms or 100Hz.
 */

class PID
{
public:
  float error = 0;
  float Kp = 0;
  float Ki = 0;
  float Kd = 0;
  float startI = 0;
  float settleError = 0;
  float settleTime = 0;
  float timeout = 0;
  float accumulatedError = 0;
  float previousError = 0;
  float output = 0;
  float timeSpentSettled = 0;
  float timeSpentRunning = 0;
  float updatePeriod = 10;

  PID(float error, float Kp, float Ki, float Kd, float startI);

  PID(float error, float Kp, float Ki, float Kd, float startI, float settleError, float settleTime, float timeout);

  PID(float error, float Kp, float Ki, float Kd, float startI, float settleError, float settleTime, float timeout, float updatePeriod);

  float compute(float error);

  bool isSettled();
};