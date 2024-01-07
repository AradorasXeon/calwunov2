//Motor control video: https://www.youtube.com/watch?v=7spK_BkMJys

#define DEBUG
#include <Wire.h> 
#include "millisTimer.hpp"
#include "communication.hpp"


#include "motors.h"

MoveSlave* MoveSlave::instance = nullptr;
MoveSlave msgSlave = MoveSlave(Z_DIRECTION_STEP_COUNT);

enum class LastMoveDirection_X : uint8_t
{
    LAST_MOVE_X_LEFT = 0,
    LAST_MOVE_X_RIGHT = 255
};

enum class LastMoveDirection_Y : uint8_t
{
    LAST_MOVE_Y_UP = 0,
    LAST_MOVE_Y_DOWN = 255
};

bool hwLimiter[2];
LastMoveDirection_X lastDirectionX;
LastMoveDirection_Y lastDirectionY;

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
  //First we check the hw limiter states
  hwLimiter[0] = digitalRead(LIMIT_X);
  hwLimiter[1] = digitalRead(LIMIT_Y);

  //if necessary we set the sw limiters
  if(hwLimiter[0])
  {
    if(lastDirectionX == LastMoveDirection_X::LAST_MOVE_X_LEFT)   limiterStates[0] = true;
    if(lastDirectionX == LastMoveDirection_X::LAST_MOVE_X_RIGHT)  limiterStates[1] = true;
  }
  else
  {
    limiterStates[0] = false;
    limiterStates[1] = false;
  }

  if(hwLimiter[1])
  {
    if(lastDirectionY == LastMoveDirection_Y::LAST_MOVE_Y_UP)     limiterStates[2] = true;
    if(lastDirectionY == LastMoveDirection_Y::LAST_MOVE_Y_DOWN)   limiterStates[3] = true;
  }
  else
  {
    limiterStates[2] = false;
    limiterStates[3] = false;
  }

  //Z SW LIMITERS only used when calibrated
  if(msgSlave.isMessageFromMasterContainsCalibState(Claw_Calibration::CLAW_CALIB_FINISHED))
  {
    if(msgSlave.getCurrentZPosition() <= 0)
    {
      limiterStates[4] = true;
    }
    else
    {
      limiterStates[4] = false;
    }

    if(msgSlave.getCurrentZPosition() >= msgSlave.getMaxZPosition())
    {
      limiterStates[5] = true;
    }
    else
    {
      limiterStates[5] = false;
    }
  }

  /*
  //DEBUG
  #ifdef DEBUG
  for(int i = 0; i<6; i++)
  {
   Serial.print(limiterStates[i], DEC);
   Serial.print("\t");
  }
  Serial.println("^^^^^^^^^^^^^^^^^^^^^****");
  #endif // DEBUG
  */

  //DEBUG end

  //Power saving
  /*
  if(msgSlave.isMessageFromMasterContainsControllState(Claw_Controll_State::CLAW_CONTROLL_STATE_IDLE))
  {
    digitalWrite(ENABLER, HIGH);
    #ifdef DEBUG
      Serial.println("STEPPERS DISABLED");
    #endif // DEBUG
  }
  else if (msgSlave.isMessageFromMasterContainsControllState(Claw_Controll_State::CLAW_CONTROLL_STATE_ERROR))
  {
    digitalWrite(ENABLER, HIGH);
    #ifdef DEBUG
      Serial.println("STEPPERS DISABLED");
    #endif // DEBUG
  }
  else
  {
    digitalWrite(ENABLER, LOW); //activates steppers
    #ifdef DEBUG
      Serial.println("STEPPERS ENABLED *****************************************");
    #endif // DEBUG
  }
  */

  //Then we see if we are in calibration mode 
  if(msgSlave.isMessageFromMasterContainsCalibState(Claw_Calibration::CLAW_CALIB_INIT))
  {
    //calibration
    while(msgSlave.isMessageFromMasterContainsCalibState(Claw_Calibration::CLAW_CALIB_INIT))
    {
      //UPWards
      if(msgSlave.isMessageFromMasterContainsControllState(Claw_Controll_State::CLAW_CONTROLL_STATE_UP))
      {
        msgSlave.decrementZPositon();
        moveClawUp(Z_DIRECTION_STEP_COUNT);
      }

      //DOWNWards
      if(msgSlave.isMessageFromMasterContainsControllState(Claw_Controll_State::CLAW_CONTROLL_STATE_DOWN))
      {
        msgSlave.incrementZPositon();
        moveClawDown(Z_DIRECTION_STEP_COUNT);
      }

      //CLAW_CALIB_TOP_STATE_IN_PROGRESS
      if(msgSlave.isMessageFromMasterContainsCalibState(Claw_Calibration::CLAW_CALIB_TOP_STATE_IN_PROGRESS))
      {
        if(msgSlave.isMessageFromMasterContainsControllState(Claw_Controll_State::CLAW_CONTROLL_STATE_BUTTON))
        {
          msgSlave.setZTopPosition();
        }
        //"saying it is done" should be on master side
      }

      //CLAW_CALIB_DOWN_STATE_IN_PROGRESS
      if(msgSlave.isMessageFromMasterContainsCalibState(Claw_Calibration::CLAW_CALIB_DOWN_STATE_IN_PROGRESS))
      {
        if(msgSlave.isMessageFromMasterContainsControllState(Claw_Controll_State::CLAW_CONTROLL_STATE_BUTTON))
        {
          msgSlave.setZBottomPosition();
        }
        //"saying it is done" should be on master side
      }

      msgSlave.setDefaultControllState(); //in case of lost connection do nothing

    }
  }

  if(msgSlave.isMessageFromMasterContainsCalibState(Claw_Calibration::CLAW_CALIB_FINISHED))
  {
    //LEFT
    if(msgSlave.isMessageFromMasterContainsControllState(Claw_Controll_State::CLAW_CONTROLL_STATE_LEFT))
    {
      lastDirectionX = LastMoveDirection_X::LAST_MOVE_X_LEFT;
      moveLeft(X_DIRECTION_STEP_COUNT);
    }

    //RIGHT
    if(msgSlave.isMessageFromMasterContainsControllState(Claw_Controll_State::CLAW_CONTROLL_STATE_RIGHT))
    {
      lastDirectionX = LastMoveDirection_X::LAST_MOVE_X_RIGHT;
      moveRight(X_DIRECTION_STEP_COUNT);
    }

    //UP
    if(msgSlave.isMessageFromMasterContainsControllState(Claw_Controll_State::CLAW_CONTROLL_STATE_UP))
    {
      lastDirectionY = LastMoveDirection_Y::LAST_MOVE_Y_UP;
      moveUp(Y_DIRECTION_STEP_COUNT);
    }

    //DOWN
    if(msgSlave.isMessageFromMasterContainsControllState(Claw_Controll_State::CLAW_CONTROLL_STATE_DOWN))
    {
      lastDirectionY = LastMoveDirection_Y::LAST_MOVE_Y_DOWN;
      moveDown(Y_DIRECTION_STEP_COUNT);
    }

    //BUTTON
    if(msgSlave.isMessageFromMasterContainsControllState(Claw_Controll_State::CLAW_CONTROLL_STATE_BUTTON))
    {
      //CLAW ACTION HERE
      if(msgSlave.isMessageFromMasterContainsCalibState(Claw_Calibration::CLAW_CALIB_DOWN_DONE) && msgSlave.isMessageFromMasterContainsCalibState(Claw_Calibration::CLAW_CALIB_TOP_DONE))
      {
        while(msgSlave.getCurrentZPosition() < msgSlave.getMaxZPosition()) //we do this until we have reached the bottom position
        {
          msgSlave.incrementZPositon();
          moveClawDown(Z_DIRECTION_STEP_COUNT);
        }

        //we should wait here until claw is closed 

        while(msgSlave.getCurrentZPosition() > 0) //we do this until the top is reached 
        {
          msgSlave.decrementZPositon();
          moveClawUp(Z_DIRECTION_STEP_COUNT);
        }

        //then goto home position 

      }
    }
  }

  msgSlave.setDefaultControllState(); //in case of lost connection do nothing

}
