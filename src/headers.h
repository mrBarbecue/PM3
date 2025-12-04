#ifndef HEADERS_H
#define HEADERS_H

#include "mbed.h"
#include "include/PESBoardPinMap.h"
#include "lib/DebounceIn/DebounceIn.h"
#include "lib\SerialStream\SerialStream.h"
#include "ThreadFlag.h"
#include <cstdint>


class DFRobotDFPlayerMini{
private:

    #define DFPLAYER_EQ_NORMAL 0
    #define DFPLAYER_EQ_POP 1
    #define DFPLAYER_EQ_ROCK 2
    #define DFPLAYER_EQ_JAZZ 3
    #define DFPLAYER_EQ_CLASSIC 4
    #define DFPLAYER_EQ_BASS 5

    #define DFPLAYER_DEVICE_U_DISK 1
    #define DFPLAYER_DEVICE_SD 2
    #define DFPLAYER_DEVICE_AUX 3
    #define DFPLAYER_DEVICE_SLEEP 4
    #define DFPLAYER_DEVICE_FLASH 5

    #define DFPLAYER_RECEIVED_LENGTH 10
    #define DFPLAYER_SEND_LENGTH 10

    //#define _DEBUG

    #define TimeOut 0
    #define WrongStack 1
    #define DFPlayerCardInserted 2
    #define DFPlayerCardRemoved 3
    #define DFPlayerCardOnline 4
    #define DFPlayerPlayFinished 5
    #define DFPlayerError 6
    #define DFPlayerUSBInserted 7
    #define DFPlayerUSBRemoved 8
    #define DFPlayerUSBOnline 9
    #define DFPlayerCardUSBOnline 10
    #define DFPlayerFeedBack 11

    #define Busy 1
    #define Sleeping 2
    #define SerialWrongStack 3
    #define CheckSumNotMatch 4
    #define FileIndexOut 5
    #define FileMismatch 6
    #define Advertise 7

    #define Stack_Header 0
    #define Stack_Version 1
    #define Stack_Length 2
    #define Stack_Command 3
    #define Stack_ACK 4
    #define Stack_Parameter 5
    #define Stack_CheckSum 7
    #define Stack_End 9

    BufferedSerial serial;

    DigitalIn BusyPin;

    unsigned long _timeOutTimer;
    unsigned long _timeOutDuration = 500;

    uint8_t _received[DFPLAYER_RECEIVED_LENGTH];
    uint8_t _sending[DFPLAYER_SEND_LENGTH] = {0x7E, 0xFF, 06, 00, 01, 00, 00, 00, 00, 0xEF};

    uint8_t _receivedIndex=0;

    void sendStack();
    void sendStack(uint8_t command);
    void sendStack(uint8_t command, uint16_t argument);
    void sendStack(uint8_t command, uint8_t argumentHigh, uint8_t argumentLow);

    void enableACK();
    void disableACK();

    void uint16ToArray(uint16_t value,uint8_t *array);

    uint16_t arrayToUint16(uint8_t *array);

    uint16_t calculateCheckSum(uint8_t *buffer);

    void parseStack();
    bool validateStack();

    uint8_t device = DFPLAYER_DEVICE_SD;

    bool baneModeActive = false;        //Changes voicecomands to the voice of bane
    const int baneModeOffset = 6;   //Offset thats added to player commands to play bane voices

public:

    explicit DFRobotDFPlayerMini(PinName RX, PinName TX, PinName BUSY);

    bool getBusyState();
    bool getBaneModeState();
    void setBaneModeState();

    typedef enum{
        startsAtOne,
        notInitialized,
        inUpperPos,
        inLowerPos,
        manualMode,
        automaticMode,
        initializing
    }FileName;

    uint8_t _handleType;
    uint8_t _handleCommand;
    uint16_t _handleParameter;
    bool _isAvailable = false;
    bool _isSending = false;

    bool handleMessage(uint8_t type, uint16_t parameter = 0);
    bool handleError(uint8_t type, uint16_t parameter = 0);

    uint8_t readCommand();

    bool begin(bool isACK = false, bool doReset = true);

    bool waitAvailable(unsigned long duration = 0);

    bool available();

    uint8_t readType();

    uint16_t read();

    void setTimeOut(unsigned long timeOutDuration);

    void next();

    void previous();

    void play(int fileNumber=1);

    void volumeUp();

    void volumeDown();

    void volume(uint8_t volume);

    void EQ(uint8_t eq);

    void loop(int fileNumber);

    void outputDevice(uint8_t device);

    void sleep();

    void reset();

    void start();

    void pause();

    void playFolder(uint8_t folderNumber, uint8_t fileNumber);

    void outputSetting(bool enable, uint8_t gain);

    void enableLoopAll();

    void disableLoopAll();

    void playMp3Folder(int fileNumber);

    void advertise(int fileNumber);

    void playLargeFolder(uint8_t folderNumber, uint16_t fileNumber);

    void stopAdvertise();

    void stop();

    void loopFolder(int folderNumber);

    void randomAll();

    void enableLoop();

    void disableLoop();

    void enableDAC();

    void disableDAC();

    int readState();

    int readVolume();

    int readEQ();

    int readFileCounts(uint8_t device);

    int readCurrentFileNumber(uint8_t device);

    int readFileCountsInFolder(int folderNumber);

    int readFileCounts();

    int readFolderCounts();

    int readCurrentFileNumber();
  
};


class Stepper
{
public:
    #define PI 3.1415926
    /**
     * @brief Constructs a new Stepper motor object.
     *
     * Initializes the stepper motor with specified step and direction pins.
     * Optionally, the steps per revolution can be set, defaulting to 200 * 16 steps.
     *
     * @param step_pin Pin to control the stepping pulses.
     * @param dir_pin Pin to control the direction of rotation.
     * @param step_per_rev Number of steps per revolution (default is 3200).
     */
    explicit Stepper(PinName step_pin, PinName dir_pin, uint16_t step_per_rev = 200 * 16);

    /**
     * @brief Destroys the Stepper object, cleaning up resources.
     */
    virtual ~Stepper();

    bool rampUp(uint8_t mainExecutionTime);   //steadily increases motorspeed to lift the jack, returns true if max speed has been reached
                                                    //needs set execution time of mainloop

    bool rampDown(uint8_t mainExecutionTime);       //steadily increases motorspeed to lower the jack, returns true if max (negative) speed has been reached
                                                    //needs set execution time of mainloop
                                                    
    bool up();      //lifts the jack with constant speed, returns 1 if top pos has been reached

    bool down();    //lowers the jack with constant speed, returns 1 if lowest pos has been reached

    /**
     * @brief Sets the internal rotation of the motor.
     *
     * This function sets the internal rotation position without moving the motor.
     *
     * @param rotations The rotation count to set (default is 0.0f).
     */
    void setInternalRotation(float rotations = 0.0f) { m_steps = static_cast<int>(rotations * m_steps_per_rev + 0.5f); };

    /**
     * @brief Sets the motor's velocity.
     *
     * Adjusts the speed of the motor's movement.
     *
     * @param velocity The new velocity to set.
     */
    void setVelocity(float velocity);

    /**
     * @brief Gets the current velocity of the motor.
     *
     * Returns the motor's velocity in rotations per second.
     *
     * @return Current velocity.
     */
    float getVelocity() const { return m_velocity; };

    /**
     * @brief Gets the current rotation count.
     *
     * Returns the current rotation count, calculated as steps divided by steps per revolution.
     *
     * @return Current rotation count.
     */
    float getRotation() const { return static_cast<float>(m_steps) / m_steps_per_rev; };


private:
    static constexpr int PULSE_MUS = 10;
    static constexpr int STEPS_MIN = std::numeric_limits<int>::min();
    static constexpr int STEPS_MAX = std::numeric_limits<int>::max();


    const uint16_t maxVelocity = 12;             //Maximal Motorvelocity (rotations per second), max allowed: 16, max recomenden: 10
    const float rampUpTime = 1000;              //Time in ms for the motor to reach velocity
    const uint16_t softwareStopPos = 10;        //Offset in rotation from the Motor to the lowest position

    const uint16_t rotationToTopPos = 600 ;     //How many rotation the Motor must spin to reach the top Position, starting from the bottom


    DigitalOut m_Step;
    DigitalOut m_Dir;

    Thread m_Thread;
    Ticker m_Ticker;
    ThreadFlag m_ThreadFlag;
    Timeout m_Timeout;

    float m_steps_per_rev;
    float m_time_step_const;

    int m_steps_setpoint;
    int m_steps;
    float m_velocity;

    int m_period_mus;  ///< Period of the step signal in microseconds.

    void step();
    void enableDigitalOutput();
    void disableDigitalOutput();
    void threadTask();
    void sendThreadFlag();
    float calcVelocity();

    
    /**
     * @brief Sets the motor's rotation to a specified position at a given velocity.
     *
     * Moves the motor to the specified rotation count at the provided velocity.
     *
     * @param rotations Target rotation position.
     * @param velocity Speed at which to reach the target rotation (default is 1.0f).
     */
    void setRotation(float rotations, float velocity = 1.0f);

    /**
     * @brief Sets the motor's rotation to a specified position.
     *
     * Moves the motor to the specified rotation count at the current velocity.
     *
     * @param rotations Target rotation position.
     */
    void setRotation(float rotations);

    /**
     * @brief Sets the motor's relative rotation at a given velocity.
     *
     * Moves the motor by a specified relative rotation amount at the provided velocity.
     *
     * @param rotations Relative rotation amount.
     * @param velocity Speed at which to reach the target rotation (default is 1.0f).
     */
    void setRotationRelative(float rotations, float velocity = 1.0f);

    /**
     * @brief Sets the motor's relative rotation.
     *
     * Moves the motor by a specified relative rotation amount at the current velocity.
     *
     * @param rotations Relative rotation amount.
     */
    void setRotationRelative(float rotations);


    /**
     * @brief Sets the motor's steps to a specific value at a given velocity.
     *
     * Moves the motor to a specific step count at the provided speed.
     *
     * @param steps Target step position.
     * @param velocity Speed at which to reach the target step position.
     */
    void setSteps(int steps, float velocity);

        /**
     * @brief Sets the internal velocity of the motor.
     *
     * This function sets the internal velocity without moving the motor.
     *
     * @param velocity The velocity to set (default is 0.0f).
     */
    void setInternalVelocity(float velocity = 0.0f) { m_velocity = velocity; };

        /**
     * @brief Gets the current step setpoint.
     *
     * Returns the target step position that the motor is set to reach.
     *
     * @return Current step setpoint.
     */
    int getStepsSetpoint() const { return m_steps_setpoint; };

    /**
     * @brief Gets the current step count.
     *
     * Returns the current step position of the motor.
     *
     * @return Current step position.
     */
    int getSteps() const { return m_steps; };


};

#endif