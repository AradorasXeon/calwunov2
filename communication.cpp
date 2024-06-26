#include "communication.hpp"
#define DEBUG


// Claw_Controll_State OPERATORS ------------------------------------------------------------------------------------

Claw_Controll_State operator|(Claw_Controll_State left, Claw_Controll_State right)
{
    return static_cast<Claw_Controll_State>(static_cast<uint8_t>(left) | static_cast<uint8_t>(right));
}

Claw_Controll_State operator&(Claw_Controll_State left, Claw_Controll_State right)
{
    return static_cast<Claw_Controll_State>(static_cast<uint8_t>(left) & static_cast<uint8_t>(right));
}

Claw_Controll_State operator~(Claw_Controll_State input)
{
    return static_cast<Claw_Controll_State>(~static_cast<uint8_t>(input));
}

// Claw_Calibration OPERATORS ------------------------------------------------------------------------------------

Claw_Calibration operator|(Claw_Calibration left, Claw_Calibration right)
{
    return static_cast<Claw_Calibration>(static_cast<uint8_t>(left) | static_cast<uint8_t>(right));
}

Claw_Calibration operator&(Claw_Calibration left, Claw_Calibration right)
{
    return static_cast<Claw_Calibration>(static_cast<uint8_t>(left) & static_cast<uint8_t>(right));
}

Claw_Calibration operator~(Claw_Calibration input)
{
    return static_cast<Claw_Calibration>(~static_cast<uint8_t>(input));
}

// MoveMaster class function implementations ------------------------------------------------------------------------------------

MoveMaster::MoveMaster() : timer(2)
{
    setDefaultControllState();
    calibDefault();
    _msgReadFromSlave.calibState = Claw_Calibration::CLAW_CALIB_IDLE_STATE;
    _msgReadFromSlave.zHeight = 0;
    _msgReadFromSlave.zHeightMax = -99000;  //this should be a big positive number
    _msgReadFromSlave.zHeightMin = 99000;   //this should be a small or negative number

    Wire.begin();
}

void MoveMaster::setLeft()
{
    _msgToSend.controlState = ~Claw_Controll_State::CLAW_CONTROLL_STATE_RIGHT & _msgToSend.controlState;
    _msgToSend.controlState = _msgToSend.controlState | Claw_Controll_State::CLAW_CONTROLL_STATE_LEFT;    
}

void MoveMaster::setRight()
{
    _msgToSend.controlState = ~Claw_Controll_State::CLAW_CONTROLL_STATE_LEFT & _msgToSend.controlState;
    _msgToSend.controlState = _msgToSend.controlState | Claw_Controll_State::CLAW_CONTROLL_STATE_RIGHT;    
}

void MoveMaster::setUp()
{
    _msgToSend.controlState = ~Claw_Controll_State::CLAW_CONTROLL_STATE_DOWN & _msgToSend.controlState;
    _msgToSend.controlState = _msgToSend.controlState | Claw_Controll_State::CLAW_CONTROLL_STATE_UP;    
}

void MoveMaster::setDown()
{
    _msgToSend.controlState = ~Claw_Controll_State::CLAW_CONTROLL_STATE_UP & _msgToSend.controlState;
    _msgToSend.controlState = _msgToSend.controlState | Claw_Controll_State::CLAW_CONTROLL_STATE_DOWN;    
}

void MoveMaster::setButton()
{
    _msgToSend.controlState = _msgToSend.controlState | Claw_Controll_State::CLAW_CONTROLL_STATE_BUTTON;    
}

void MoveMaster::setDefaultControllState()
{
    _msgToSend.controlState = Claw_Controll_State::CLAW_CONTROLL_STATE_IDLE;    
}

void MoveMaster::setError()
{
    _msgToSend.controlState = _msgToSend.controlState | Claw_Controll_State::CLAW_CONTROLL_STATE_ERROR;    
}

void MoveMaster::calibInit()
{
    _msgToSend.calibState = Claw_Calibration::CLAW_CALIB_INIT;
}

void MoveMaster::calibStratTop()
{
    _msgToSend.calibState = ~Claw_Calibration::CLAW_CALIB_TOP_DONE & _msgToSend.calibState;
    _msgToSend.calibState = _msgToSend.calibState | Claw_Calibration::CLAW_CALIB_TOP_STATE_IN_PROGRESS;
}

void MoveMaster::calibFinishTop()
{
    _msgToSend.calibState = ~Claw_Calibration::CLAW_CALIB_TOP_STATE_IN_PROGRESS & _msgToSend.calibState;
    _msgToSend.calibState = _msgToSend.calibState | Claw_Calibration::CLAW_CALIB_TOP_DONE;
}

void MoveMaster::calibStartDown()
{
    _msgToSend.calibState = ~Claw_Calibration::CLAW_CALIB_DOWN_DONE & _msgToSend.calibState;
    _msgToSend.calibState = _msgToSend.calibState | Claw_Calibration::CLAW_CALIB_DOWN_STATE_IN_PROGRESS;
}

void MoveMaster::calibFinishDown()
{
    _msgToSend.calibState = ~Claw_Calibration::CLAW_CALIB_DOWN_STATE_IN_PROGRESS & _msgToSend.calibState;
    _msgToSend.calibState = _msgToSend.calibState | Claw_Calibration::CLAW_CALIB_DOWN_DONE;
}

void MoveMaster::calibDone()
{
    _msgToSend.calibState = ~Claw_Calibration::CLAW_CALIB_INIT & _msgToSend.calibState;
    _msgToSend.calibState = _msgToSend.calibState | Claw_Calibration::CLAW_CALIB_FINISHED;
}

void MoveMaster::calibDefault()
{
    _msgToSend.calibState = Claw_Calibration::CLAW_CALIB_IDLE_STATE;
}

void MoveMaster::calibError()
{
    _msgToSend.calibState = _msgToSend.calibState | Claw_Calibration::CLAW_CALIB_BAD;
}

/// @brief sends the message(calib state and controller state) on i2c from the master to the slave
void MoveMaster::sendMessageToSlave()
{
    Wire.beginTransmission(I2C_MOTOR_CTRL_ADDRESS);
    Wire.write(static_cast<uint8_t>(_msgToSend.calibState));
    Wire.write(static_cast<uint8_t>(_msgToSend.controlState));
    Wire.endTransmission(true);
    timer.doDelay();
}

/// @brief reads in the status of the slave, e.g.: z postion and the calib state of the slave
void MoveMaster::readMessageFromSlave()
{
    Wire.requestFrom(I2C_MOTOR_CTRL_ADDRESS, sizeof(MessageFromSlave), true);
    Wire.readBytes((uint8_t*)&_msgReadFromSlave, sizeof(MessageFromSlave));
    #ifdef DEBUG
        Serial.print("calibState: ");
        Serial.println((uint8_t)_msgReadFromSlave.calibState, BIN);
        Serial.print("zHeight: ");
        Serial.println(_msgReadFromSlave.zHeight, DEC);
        Serial.print("zHeightMax: ");
        Serial.println(_msgReadFromSlave.zHeightMax, DEC);
        Serial.print("zHeightMin: ");
        Serial.println(_msgReadFromSlave.zHeightMin, DEC);
    #endif // DEBUG
    timer.doDelay();
}

/// @brief supposed to be in the past tense, this function sees what was the calib state in the lastly read message
/// @param searchedCalibState the state to check in the last message
/// @return true if the searchedCalibState was contained in the last message
bool MoveMaster::isReadCalibStateContains(Claw_Calibration searchedCalibState)
{
    Claw_Calibration temp = _msgReadFromSlave.calibState & searchedCalibState;
    if(temp == searchedCalibState)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/// @brief checks if the button is PRESSED in the toBeSentMsg
/// @return true if it is true in the to be sent message
bool MoveMaster::wasButtonPressed()
{
    Claw_Controll_State temp = _msgToSend.controlState & Claw_Controll_State::CLAW_CONTROLL_STATE_BUTTON;
    if (temp == Claw_Controll_State::CLAW_CONTROLL_STATE_BUTTON)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/// @brief to see if we are at top position, should be used after proper calibration
/// @return true if according to the lastly read message z is at the top
bool MoveMaster::isZatTop()
{
    if(_msgReadFromSlave.zHeight <= _msgReadFromSlave.zHeightMin) //safer with <=
        return true;
    else
        return false;
}

/// @brief to see if we are at bottom position, should be used after proper calibration
/// @return true if according to the lastly read message z is at the bottom
bool MoveMaster::isZatBottom()
{
    if(_msgReadFromSlave.zHeight >= _msgReadFromSlave.zHeightMax) //safer with >=
        return true;
    else
        return false;
}

bool MoveMaster::isClawAtHome()
{
    return _msgReadFromSlave.isHome;
}

// MoveSlave class function implementations ------------------------------------------------------------------------------------

/// @brief First outside the setup function: MoveSlave* MoveSlave::instance = nullptr; then init it normally then in the setup first give the instance: nameyougave.instance = &nameyougave then use these: Wire.onReceive(MoveSlave::readMessageFromMaster); and Wire.onRequest(&(msgSlave.instance->replyToMaster));
MoveSlave::MoveSlave(int32_t* pointerToGlobalZPos) : timer(2)
{   
    setDefaultControllState();
    calibDefault();
    _currZpos = pointerToGlobalZPos;

    Wire.begin(I2C_MOTOR_CTRL_ADDRESS);
}

/// @brief Should be used like: Wire.onReceive(MoveSlave::readMessageFromMaster);
/// @param byteCount this comes from the onReceive function
void MoveSlave::readMessageFromMaster(int byteCount)
{
    if(instance != nullptr)
    {
        instance->readMsg(byteCount);
    }
}

/// @brief This sets the _msgFromMaster.controlState to DEFAULT, SHOULD BE USED IF COMM IS LOST
void MoveSlave::setDefaultControllState()
{
    _msgFromMaster.controlState = Claw_Controll_State::CLAW_CONTROLL_STATE_IDLE;    
}

/// @brief This sets the _msgFromMaster.calibState to DEFAULT, SHOULD BE USED IF COMM IS LOST
void MoveSlave::calibDefault()
{
    _msgFromMaster.calibState = Claw_Calibration::CLAW_CALIB_IDLE_STATE;
}

/// @brief This sets the OUTGOING calibration message to error
void MoveSlave::calibError()
{
    _msgToSend.calibState = _msgToSend.calibState | Claw_Calibration::CLAW_CALIB_BAD;
}

/// @brief to read the last message
/// @return the message
MessageFromMaster MoveSlave::getMessageFromMaster()
{
    return _msgFromMaster;
}

/// @brief this function sees what was the calib state is supposed to be based on the lastly read message from master
/// @param searchedCalibState the state to check in the last message
/// @return true if the searchedCalibState was contained in the last message
bool MoveSlave::isMessageFromMasterContainsCalibState(Claw_Calibration searchedCalibState)
{
    Claw_Calibration temp = _msgFromMaster.calibState & searchedCalibState;
    if(temp == searchedCalibState)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/// @brief this function sees what was the Controll state is supposed to be based on the lastly read message from master
/// @param searchedControllState the state to check in the last message
/// @return true if the searchedControllState was contained in the last message
bool MoveSlave::isMessageFromMasterContainsControllState(Claw_Controll_State searchedControllState)
{
    Claw_Controll_State temp = _msgFromMaster.controlState & searchedControllState;
    if(temp == searchedControllState)
    {
        return true;
    }
    else
    {
        return false;
    }
}

int32_t MoveSlave::getCurrentZPosition()
{
    return *_currZpos;
}

/// @brief should be set when incoming msg is set to top calib done for first time
void MoveSlave::setZTopPosition()
{
    _zTopPosition = getCurrentZPosition();
}

/// @brief should be set when incoming msg is set to top calib down for first time
void MoveSlave::setZBottomPosition()
{
    _zBottomPosition = getCurrentZPosition();
}

/// @brief says we are at home
void MoveSlave::setClawHomePosition()
{
    _isClawAtHome = true;
}

/// @brief says we are not at home
void MoveSlave::unsetClawFromHome()
{
    _isClawAtHome = false;
}

void MoveSlave::readMsg(int byteCount)
{
    _msgFromMaster.calibState = (Claw_Calibration)Wire.read();
    _msgFromMaster.controlState = (Claw_Controll_State)Wire.read();
}

int32_t MoveSlave::getZPosBottom()
{
    return _zBottomPosition;
}

int32_t MoveSlave::getZPosTop()
{
    return _zTopPosition;
}

void MoveSlave::replyToMaster()
{
    instance->_msgToSend.calibState = instance->_msgFromMaster.calibState; //this is the lastly read message from the Master
    instance->_msgToSend.zHeight    = *(instance->_currZpos);
    instance->_msgToSend.zHeightMax = instance->_zBottomPosition;
    instance->_msgToSend.zHeightMin = instance->_zTopPosition;
    instance->_msgToSend.isHome     = instance->_isClawAtHome;

    #ifdef DEBUG
        Serial.print("calibState: ");
        Serial.println((uint8_t)instance->_msgToSend.calibState, BIN);
        Serial.print("zHeight: ");
        Serial.println(instance->_msgToSend.zHeight, DEC);
        Serial.print("zHeightMax: ");
        Serial.println(instance->_msgToSend.zHeightMax, DEC);
        Serial.print("zHeightMin: ");
        Serial.println(instance->_msgToSend.zHeightMin, DEC);
    #endif // DEBUG

    Wire.write((byte*)&(instance->_msgToSend), sizeof(MessageFromSlave));
}