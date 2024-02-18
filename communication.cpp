#include "communication.hpp"

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
    _msgReadFromSlave.zHeight = -2500;
    _msgReadFromSlave.zHeightMax = -1;

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
    byte buff[9];
    Wire.requestFrom(I2C_MOTOR_CTRL_ADDRESS, sizeof(MessageFromSlave), true);
    Wire.readBytes(buff, 9);
    _msgReadFromSlave.calibState = (Claw_Calibration)buff[0];
    _msgReadFromSlave.zHeight =     int32_t(buff[1]<<24) | int32_t(buff[2]<<16) | int32_t(buff[3]<<8) | int32_t(buff[4]);
    _msgReadFromSlave.zHeightMax =  int32_t(buff[5]<<24) | int32_t(buff[6]<<16) | int32_t(buff[7]<<8) | int32_t(buff[8]);
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
    if(_msgReadFromSlave.zHeight <= 0) //safer with <=
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