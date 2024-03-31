//Motor control video: https://www.youtube.com/watch?v=7spK_BkMJys

#define DEBUG
#include <Wire.h> 
#include "millisTimer.hpp"
#include "communication.hpp"


#include "motors.h"

#ifdef DEBUG
  #define DEBUG_PRINTLN(msg) Serial.println(msg)
#else
  #define DEBUG_PRINTLN(msg)
#endif // DEBUG

#define TIME_CLAW_CLOSE_TIME_MS 1000    //for now

MoveSlave* MoveSlave::instance = nullptr;
MoveSlave msgSlave = MoveSlave(&globalZposition);

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

MillisTimer timerClawClose(TIME_CLAW_CLOSE_TIME_MS);

void setup() 
{
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ENABLER_X, OUTPUT);
  pinMode(ENABLER_Y, OUTPUT);
  pinMode(ENABLER_Z, OUTPUT);


  pinMode(STEP_PIN_X, OUTPUT);
  pinMode(STEP_PIN_Y, OUTPUT);
  pinMode(STEP_PIN_Z, OUTPUT);

  pinMode(STEP_DIR_X, OUTPUT);
  pinMode(STEP_DIR_Y, OUTPUT);
  pinMode(STEP_DIR_Z, OUTPUT);

  pinMode(LIMIT_X, INPUT);
  pinMode(LIMIT_Y, INPUT);

  digitalWrite(ENABLER_X, LOW);
  digitalWrite(ENABLER_Y, LOW);


  limiterStates[0] = 0;   //moveLeft
  limiterStates[1] = 0;   //moveRight
  limiterStates[2] = 0;   //moveUp
  limiterStates[3] = 0;   //moveDown
  limiterStates[4] = 0;   //moveClawUp
  limiterStates[5] = 0;   //moveClawDown
  
  msgSlave.instance = &msgSlave;
  Wire.onReceive(MoveSlave::readMessageFromMaster);
  #ifdef DEBUG
    Serial.begin(115200);
  #endif // DEBUG
}

void limiterStateCheckerUpdater()
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
    if(msgSlave.getCurrentZPosition() <= msgSlave.getZPosTop())       //Z pos gets bigger towards bottom
    {
      limiterStates[4] = true;
    }
    else
    {
      limiterStates[4] = false;
    }

    if(msgSlave.getCurrentZPosition() >= msgSlave.getZPosBottom())    //Z pos gets bigger towards bottom
    {
      limiterStates[5] = true;
    }
    else
    {
      limiterStates[5] = false;
    }
  }
}

void loop() 
{
  limiterStateCheckerUpdater();
  
  //Then we see if we are in calibration mode 
  if(msgSlave.isMessageFromMasterContainsCalibState(Claw_Calibration::CLAW_CALIB_INIT))
  {
    digitalWrite(ENABLER_Z, LOW);

    //calibration
    while(msgSlave.isMessageFromMasterContainsCalibState(Claw_Calibration::CLAW_CALIB_INIT))
    {
      limiterStateCheckerUpdater();

      //UPWards
      if(msgSlave.isMessageFromMasterContainsControllState(Claw_Controll_State::CLAW_CONTROLL_STATE_UP))
      {
        moveClawUp(Z_DIRECTION_STEP_COUNT);
      }

      //DOWNWards
      if(msgSlave.isMessageFromMasterContainsControllState(Claw_Controll_State::CLAW_CONTROLL_STATE_DOWN))
      {
        moveClawDown(Z_DIRECTION_STEP_COUNT);
      }

      //CLAW_CALIB_TOP_STATE_IN_PROGRESS
      if(msgSlave.isMessageFromMasterContainsCalibState(Claw_Calibration::CLAW_CALIB_TOP_STATE_IN_PROGRESS))
      {
        if(msgSlave.isMessageFromMasterContainsControllState(Claw_Controll_State::CLAW_CONTROLL_STATE_BUTTON))
        {
          msgSlave.setZTopPosition();
          DEBUG_PRINTLN("Z TOP set.");
        }
        //"saying it is done" should be on master side
      }

      //CLAW_CALIB_DOWN_STATE_IN_PROGRESS
      if(msgSlave.isMessageFromMasterContainsCalibState(Claw_Calibration::CLAW_CALIB_DOWN_STATE_IN_PROGRESS))
      {
        if(msgSlave.isMessageFromMasterContainsControllState(Claw_Controll_State::CLAW_CONTROLL_STATE_BUTTON))
        {
          msgSlave.setZBottomPosition();  //set the bottom position
          DEBUG_PRINTLN("Z BOTTOM set.");
          //go to the top after that
          while(msgSlave.getCurrentZPosition() > msgSlave.getZPosTop()) //we do this until the top is reached 
          {
            moveClawUp(Z_DIRECTION_STEP_COUNT);
            DEBUG_PRINTLN("CALIB: pulling up claw.");
          }
        }
        //"saying it is done" should be on master side
      }

      msgSlave.setDefaultControllState(); //in case of lost connection do nothing

    }

    digitalWrite(ENABLER_Z, HIGH);

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
      digitalWrite(ENABLER_Z, LOW);
      digitalWrite(ENABLER_X, HIGH);
      digitalWrite(ENABLER_Y, HIGH);

      //disabling Z SW limiters, should be watched "manually"
      limiterStates[4] = false;
      limiterStates[5] = false;


      if(msgSlave.isMessageFromMasterContainsCalibState(Claw_Calibration::CLAW_CALIB_DOWN_DONE) && msgSlave.isMessageFromMasterContainsCalibState(Claw_Calibration::CLAW_CALIB_TOP_DONE))
      {
        while(msgSlave.getCurrentZPosition() < msgSlave.getZPosBottom()) //we do this until we have reached the bottom position
        {
          moveClawDown(Z_DIRECTION_STEP_COUNT);
          DEBUG_PRINTLN("Play: going down for prize.");
        }

        //we should wait here until claw is closed 
        timerClawClose.doDelay();

        while(msgSlave.getCurrentZPosition() > msgSlave.getZPosTop()) //we do this until the top is reached 
        {
          moveClawUp(Z_DIRECTION_STEP_COUNT);
          DEBUG_PRINTLN("Play: pulling up claw.");
        }

        //then goto home position 
        digitalWrite(ENABLER_X, LOW);
        digitalWrite(ENABLER_Y, LOW);
        limiterStateCheckerUpdater();

        //go to most left position (X)
        while(!limiterStates[0])
        {
          lastDirectionX = LastMoveDirection_X::LAST_MOVE_X_LEFT;
          moveLeft(X_DIRECTION_STEP_COUNT);
          limiterStateCheckerUpdater();
          DEBUG_PRINTLN("Play: going left HOME.");
        }

        //go to most bottom position (Y)
        while(!limiterStates[3])
        {
          lastDirectionY = LastMoveDirection_Y::LAST_MOVE_Y_DOWN;
          moveDown(Y_DIRECTION_STEP_COUNT);
          limiterStateCheckerUpdater();
          DEBUG_PRINTLN("Play: going right HOME.");
        }

        //maybe put down claw?

        //wait for prize drop

      }

      digitalWrite(ENABLER_Z, HIGH);
      digitalWrite(ENABLER_X, LOW);
      digitalWrite(ENABLER_Y, LOW);


    }
  }

  msgSlave.setDefaultControllState(); //in case of lost connection do nothing

}
