#ifndef MOTORS_H_2023_01_06
#define MOTORS_H_2023_01_06

#include "/home/krisztian/arduino/clawnanov2/timer.h"             


#define STEP_PIN_X 2
#define STEP_PIN_Y 3
#define STEP_PIN_Z 4

#define STEP_DIR_X 5
#define STEP_DIR_Y 6
#define STEP_DIR_Z 7

#define ENABLER 8

#define LIMIT_X 9
#define LIMIT_Y 10
#define LIMIT_Z 11

#define X_DIRECTION_STEP_COUNT 30
#define Y_DIRECTION_STEP_COUNT 40
#define Z_DIRECTION_STEP_COUNT 250//120

//suggestion in the video: 700-3000, this sets the amount of microseconds between triggering the stepper's input signal

#define X_MICRO_TIME 3000
#define Y_MICRO_TIME 3000
#define Z_MICRO_TIME 500

Timer xTimer(X_MICRO_TIME);
Timer yTimer(Y_MICRO_TIME);
Timer zTimer(Z_MICRO_TIME);

bool limiterStates[6];        //false or 0 means NOT TRIGGERED THIS IS ONLY THE SW based limiter
int32_t globalZposition = 0;  //for Z SW limiters

void moveLeft(uint16_t stepCount)
{
  digitalWrite(STEP_DIR_X, HIGH);
  for(int i = 0; i<stepCount; i++)
  {
    if (limiterStates[0] != 0) break;
    digitalWrite(STEP_PIN_X, HIGH);
    xTimer.doDelay();
    digitalWrite(STEP_PIN_X, LOW);
    xTimer.doDelay();
  }
}

void moveRight(uint16_t stepCount)
{
  digitalWrite(STEP_DIR_X, LOW);
  for(int i = 0; i<stepCount; i++)
  {
    if (limiterStates[1] != 0) break;
    digitalWrite(STEP_PIN_X, HIGH);
    xTimer.doDelay();
    digitalWrite(STEP_PIN_X, LOW);
    xTimer.doDelay();
  }
}

void moveUp(uint16_t stepCount)
{
  digitalWrite(STEP_DIR_Y, HIGH);
  for(int i = 0; i<stepCount; i++)
  {
    if (limiterStates[2] != 0) break;
    digitalWrite(STEP_PIN_Y, HIGH);
    yTimer.doDelay();
    digitalWrite(STEP_PIN_Y, LOW);
    yTimer.doDelay();
  }
}

void moveDown(uint16_t stepCount)
{
  digitalWrite(STEP_DIR_Y, LOW);
  for(int i = 0; i<stepCount; i++)
  {
    if (limiterStates[3] != 0) break;
    digitalWrite(STEP_PIN_Y, HIGH);
    yTimer.doDelay();
    digitalWrite(STEP_PIN_Y, LOW);
    yTimer.doDelay();
  }
}

void moveClawUp(uint16_t stepCount)
{
  digitalWrite(STEP_DIR_Z, LOW);
  for(int i = 0; i<stepCount; i++)
  {
    if (limiterStates[4] != 0) break;
    digitalWrite(STEP_PIN_Z, HIGH);
    zTimer.doDelay();
    digitalWrite(STEP_PIN_Z, LOW);
    zTimer.doDelay();
    globalZposition--;
  }
}

void moveClawDown(uint16_t stepCount)
{
  digitalWrite(STEP_DIR_Z, HIGH);
  for(int i = 0; i<stepCount; i++)
  {
    if (limiterStates[5] != 0) break;
    digitalWrite(STEP_PIN_Z, HIGH);
    zTimer.doDelay();
    digitalWrite(STEP_PIN_Z, LOW);
    zTimer.doDelay();
    globalZposition++;
  }
}

#endif // !MOTORS_H_2023_01_06