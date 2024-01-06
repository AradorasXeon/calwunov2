//Motor control video: https://www.youtube.com/watch?v=7spK_BkMJys

#define DEBUG
#include <Wire.h> //I don't understand why i need this one here and why at otherplaces the other is good
#include "millisTimer.hpp"
#include "communication.hpp"


#include "motors.h"

MoveSlave* MoveSlave::instance = nullptr;
MoveSlave msgSlave = MoveSlave(Z_DIRECTION_STEP_COUNT);


void setup() 
{
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ENABLER, OUTPUT);

  pinMode(STEP_PIN_X, OUTPUT);
  pinMode(STEP_PIN_Y, OUTPUT);
  pinMode(STEP_PIN_Z, OUTPUT);

  pinMode(STEP_DIR_X, OUTPUT);
  pinMode(STEP_DIR_Y, OUTPUT);
  pinMode(STEP_DIR_Z, OUTPUT);

  pinMode(LIMIT_X, INPUT);
  pinMode(LIMIT_Y, INPUT);
  pinMode(LIMIT_Z, INPUT);  //not used

  digitalWrite(ENABLER, LOW);

  limiterStates[0] = 0;   //moveLeft
  limiterStates[1] = 0;   //moveRight
  limiterStates[2] = 0;   //moveUp
  limiterStates[3] = 0;   //moveDown
  limiterStates[4] = 0;   //moveClawUp
  limiterStates[5] = 0;   //moveClawDown
  
  msgSlave.instance = &msgSlave;
  Wire.onReceive(MoveSlave::readMessageFromMaster);
  Wire.onRequest(&(msgSlave.instance->replyToMaster));

  //Debug:
  #ifdef DEBUG
  Serial.begin(115200);
  Serial.println("SETUP RAN.");
  #endif // DEBUG
}







void loop() 
{

}


