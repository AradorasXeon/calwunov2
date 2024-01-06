#ifndef COMMUNICATION_MOVEMENT_2023_01_05
#define COMMUNICATION_MOVEMENT_2023_01_05

#include <Wire.h>
#include "millisTimer.hpp"


#define I2C_MOTOR_CTRL_ADDRESS 0xF2

enum class Claw_Controll_State : uint8_t
{
    CLAW_CONTROLL_STATE_IDLE    = 0b00000000,
    CLAW_CONTROLL_STATE_BUTTON  = 0b10000000,
    CLAW_CONTROLL_STATE_LEFT    = 0b00000001,
    CLAW_CONTROLL_STATE_RIGHT   = 0b00000010,
    CLAW_CONTROLL_STATE_UP      = 0b00000100,
    CLAW_CONTROLL_STATE_DOWN    = 0b00001000,
    CLAW_CONTROLL_STATE_ERROR   = 0b00010000
};

enum class Claw_Calibration : uint8_t
{
    CLAW_CALIB_IDLE_STATE =             0b00000000,
    CLAW_CALIB_INIT =                   0b00000001,
    CLAW_CALIB_TOP_STATE_IN_PROGRESS =  0b00000010,
    CLAW_CALIB_DOWN_STATE_IN_PROGRESS = 0b00000100,
    CLAW_CALIB_FINISHED =               0b00010000,
    CLAW_CALIB_BAD =                    0b00100000,
    CLAW_CALIB_TOP_DONE =               0b01000000,
    CLAW_CALIB_DOWN_DONE =              0b10000000
};

struct MessageFromMaster
{
    Claw_Calibration calibState;
    Claw_Controll_State controlState;
};

struct MessageFromSlave
{
    Claw_Calibration calibState;
    uint32_t zHeight;
    uint32_t zHeightMax;
};
class MoveMaster
{
    public:
    MoveMaster();

    void setLeft();
    void setRight();
    void setUp();
    void setDown();
    void setButton();
    void setDefaultControllState();
    void setError();

    void calibInit();
    void calibStratTop();
    void calibFinishTop();
    void calibStartDown();
    void calibFinishDown();
    void calibDone();
    void calibDefault();
    void calibError();

    void sendMessageToSlave();
    void readMessageFromSlave();
    

    bool isReadCalibStateContains(Claw_Calibration searchedCalibState);
    bool wasButtonPressed();
    bool isZatTop();
    bool isZatBottom();

    private:
    MessageFromMaster _msgToSend;
    MessageFromSlave _msgReadFromSlave;

    MillisTimer timer;

};

class MoveSlave
{
    public:
    MoveSlave(int32_t zStepRate);

    static void readMessageFromMaster(int byteCount);
    static MoveSlave* instance; //for wire lib hack

    void setDefaultControllState();
    void calibDefault();

    void calibError();

    MessageFromMaster getMessageFromMaster(); //might not need this
    bool isMessageFromMasterContainsCalibState(Claw_Calibration searchedCalibState);
    bool isMessageFromMasterContainsControllState(Claw_Controll_State searchedControllState);


    int32_t getCurrentZPosition();
    int32_t getMaxZPosition();

    void setZTopPosition();
    void setZBottomPosition();
    void incrementZPositon();
    void decrementZPositon();

    static void replyToMaster();

    private:
    void readMsg(int byteCount);

    MessageFromMaster _msgFromMaster;
    MessageFromSlave _msgToSend;
    int32_t _zStepRate;
    int32_t _zCurrentPosition; //0 should be Top position
    int32_t _zHeightBottom;

    MillisTimer timer;

};

#endif // !COMMUNICATION_MOVEMENT_2023_01_05