#include "vex.h"

/**
 * PID constructor with P, I, D, and startI.
 * Starti keeps the I term at 0 until error is less than startI.
 * 
 * @param error Difference in desired and current position.
 * @param Kp Proportional constant.
 * @param Ki Integral constant.
 * @param Kd Derivative constant.
 * @param startI Maximum error to start integrating.
 */

PID::PID(float error, float Kp, float Ki, float Kd, float startI) :
  error(error),
  Kp(Kp),
  Ki(Ki),
  Kd(Kd),
  startI(startI)
{};

/**
 * PID constructor with settling inputs.
 * The settling system works like this: The robot is settled
 * when error is less than settleError for a duration of settleTime,
 * or if the function has gone on for longer than timeout. Otherwise
 * it is not settled. Starti keeps the I term at 0 until error is less 
 * than startI.
 * 
 * @param error Difference in desired and current position.
 * @param Kp Proportional constant.
 * @param Ki Integral constant.
 * @param Kd Derivative constant.
 * @param startI Maximum error to start integrating.
 * @param settleError Maximum error to be considered settled.
 * @param settleTime Minimum time to be considered settled.
 * @param timeout Time after which to give up and move on.
 */

PID::PID(float error, float Kp, float Ki, float Kd, float startI, float settleError, float settleTime, float timeout) :
  error(error),
  Kp(Kp),
  Ki(Ki),
  Kd(Kd),
  startI(startI),
  settleError(settleError),
  settleTime(settleTime),
  timeout(timeout)
{};

/**
 * PID constructor with custom update period.
 * The default update period is 10ms, but if you want to run
 * a faster or slower loop, you need to let the settler know.
 * 
 * @param error Difference in desired and current position.
 * @param Kp Proportional constant.
 * @param Ki Integral constant.
 * @param Kd Derivative constant.
 * @param startI Maximum error to start integrating.
 * @param settleError Maximum error to be considered settled.
 * @param settleTime Minimum time to be considered settled.
 * @param timeout Time after which to give up and move on.
 * @param updatePeriod Loop delay time in ms.
 */

PID::PID(float error, float Kp, float Ki, float Kd, float startI, 
float settleError, float settleTime, float timeout, float updatePeriod) :
  error(error),
  Kp(Kp),
  Ki(Ki),
  Kd(Kd),
  startI(startI),
  settleError(settleError),
  settleTime(settleTime),
  timeout(timeout),
  updatePeriod(updatePeriod)
{};

/**
 * Computes the output power based on the error.
 * Typical PID calculation with some optimizations: When the robot crosses
 * error=0, the i-term gets reset to 0. And, of course, the robot only
 * accumulates i-term when error is less than startI. Read about these at
 * https://georgegillard.com/resources/documents.
 * 
 * @param error Difference in desired and current position.
 * @return Output power.
 */

float PID::compute(float error){
  if (fabs(error) < startI){
    accumulatedError+=error;
  }
  // Checks if the error has crossed 0, and if it has, it eliminates the integral term.
  if ((error>0 && previousError<0)||(error<0 && previousError>0)){ 
    accumulatedError = 0; 
  }

  output = Kp*error + Ki*accumulatedError + Kd*(error-previousError);

  previousError=error;

  if(fabs(error)<settleError){
    timeSpentSettled+=10;
  } else {
    timeSpentSettled = 0;
  }

  timeSpentRunning+=10;

  return output;
}

/**
 * Computes whether or not the movement has settled.
 * The robot is considered settled when error is less than settleError 
 * for a duration of settleTime, or if the function has gone on for 
 * longer than timeout. Otherwise it is not settled.
 * 
 * @return Whether the movement is settled.
 */

bool PID::isSettled(){
  if (timeSpentRunning>timeout && timeout != 0){
    return(true);
  } // If timeout does equal 0, the move will never actually time out. Setting timeout to 0 is the 
    // equivalent of setting it to infinity.
  if (timeSpentSettled>settleTime){
    return(true);
  }
  return(false);
}