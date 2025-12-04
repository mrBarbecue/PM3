#include "headers.h"
#include <cstdint>

#define _debug
//#define _playerEnabled

//Pins for Stepper Motor
#define STEP PB_14
#define DIR PC_4

//Pins for MP3-Player
#define BUSY PB_2
#define RX PA_10
#define TX PA_9

//Instantiates Inputs
DebounceIn UserButton(PC_13);

DigitalIn LimitSwitch(PC_8);
DigitalIn UpSwitch(PC_10);
DigitalIn DownSwitch(PC_11);

DigitalIn HandleSensor(PB_12);

//Instances Outputs
DigitalOut EnableStepper(PC_6);
DigitalOut Solenoid(PB_7);

bool executeMainTask = false;   //Stores whether the main should be executed
bool resetAll = false;          //Edge to reset the variables

bool motorInitialized = false;      //if true, motor is initialized

//Function is called when the UserButton is pressed
void executeMainFunction(){  
    //Switches executeMainTask when the UserButton is pressed
    executeMainTask = !executeMainTask;
    //If a positive edge is detected, resetAll is set to true
    if(executeMainTask)
        resetAll = true;
}

int main(){
    //Function is called when the UserButton is pressed
    UserButton.fall(&executeMainFunction);

    LimitSwitch.mode(PullDown);
    UpSwitch.mode(PullDown);
    DownSwitch.mode(PullDown);

    bool toggleModeDetected = false;    //detects if the mode (manual or automatic) has been changed
    bool modeState = false;             //manual=1 or automatic=0

    uint16_t solenoidReleaseCounter = 0;        //Counts the loops until the solenoid gets release
    const uint16_t solenoidReleaseDelay = 1500; //Delay between releasing the down switch and releasing the solenoid in ms

    uint16_t solenoidHoldCounter = 0;           //Counts the loops until the the carjack can be driven down
    const uint16_t driveDownDelay = 400;        //Delay between pulling the solenoid back and driving the carjack down in ms

    const uint8_t mainTaskPeriod = 255;   //Executiontime of main whileloop
    Timer MainTaskTimer;                 //Creates MainTaskTimer Object;

    bool edgedetectionVoiceModes = false;   //stores the state of the Voicemode for edgedetection

    //Starts the Timer
    MainTaskTimer.start();

    #ifdef _playerEnabled
        //Creates objects, as only one object of each class is needed, they are called the same as their classes
        DFRobotDFPlayerMini Player(RX, TX, BUSY);
        bool PlayerStatusOK = Player.begin(true, true);  // ACK on, reset

        if (!PlayerStatusOK) {
            printf("DFPlayer could not be initialized!\n");
            
        }
        else{
            printf("DFPlayer started.\n");
        }
    #endif

    
    const uint16_t stepsPerRev = 400;           //How many steps the motor perfomes to do one revolution, depends on dipswitches
    Stepper Stepper(STEP, DIR, stepsPerRev);

    Stepper.setVelocity(0);

    while(true){
        MainTaskTimer.reset();  //resets the maintasktimer

        //voicemode toggle
        if(UpSwitch.read() and DownSwitch.read()){
            if(!edgedetectionVoiceModes){
            edgedetectionVoiceModes = true;
            #ifdef _playerEnabled
                Player.setBaneModeState();
            #endif
            }
        }
        else{
           edgedetectionVoiceModes = false; 
        }

        //detects Modechange
        if(HandleSensor.read() != modeState){
            modeState = !modeState;
            toggleModeDetected = true;
        }
        else{
            toggleModeDetected = false;
        }

        //Main switch, Manual or Automatic Mode
        switch(HandleSensor.read()){
            case true: 
                #ifdef _debug
                    printf("Manual\n");
                #endif
                //Manual mode
                Stepper.setVelocity(0);
                EnableStepper.write(false);  //disbales the stepper
                Solenoid.write(false);      //Solenoid in resting position

                motorInitialized = false;   //Resets the motorInitialized flag

                //When the mode has changed to "manual" play that mp3
                #ifdef _playerEnabled
                    if(toggleModeDetected) Player.play(DFRobotDFPlayerMini::manualMode);
                    if(Stepper.down()) Player.play(DFRobotDFPlayerMini::inLowerPos);
                #endif
                
                break;

            case false:
                #ifdef _debug
                    printf("Automatic\n");
                #endif

                EnableStepper.write(true);                 //enables the stepper

                //When the mode has changed to "automatic" play that mp3
                #ifdef _playerEnabled
                    if(toggleModeDetected) Player.play(DFRobotDFPlayerMini::automaticMode);
                #endif

                if(!LimitSwitch.read()){
                    //Sets the stepcounter to 0 when the LimitSwitch is !high
                    Stepper.setInternalRotation(0);
                    motorInitialized = true;
                }
                if(DownSwitch.read()){
                    #ifdef _debug
                        printf("Down Switch, pull Solenoid back\n");
                    #endif
                    //Pulls the Solenoid back
                    Solenoid.write(true);
                    if(solenoidHoldCounter >= driveDownDelay / mainTaskPeriod){ //Delay to pull the solenoid back
                        //ramp up the motor
                        #ifdef _debug
                            printf("Motorramp downwards\n");
                        #endif
                        if(Stepper.rampDown(mainTaskPeriod)){
                            //if speed is reached, drive downwards with full speed
                            //if endpos is reached, stop
                            #ifdef _debug
                                printf("Downwards full speed\n");
                            #endif
                            if(Stepper.down()){
                                #ifdef _debug
                                    printf("In lower pos\n");
                                #endif
                                #ifdef _playerEnabled
                                    Player.play(DFRobotDFPlayerMini::inLowerPos);
                                #endif
                                
                            }
                        }
                    }
                    else solenoidHoldCounter++;
                }
                else if(UpSwitch.read()){
                    #ifdef _debug
                        printf("Up Switch\n");
                    #endif
                    if(!motorInitialized){
                        //motor not initialized
                        #ifdef _debug
                            printf("Motor not initialized\n");
                        #endif
                        #ifdef _playerEnabled
                            Player.play(DFRobotDFPlayerMini::notInitialized);
                        #endif
                    }
                    else if(Stepper.rampUp(mainTaskPeriod)){
                        Solenoid.write(false);
                        
                        //if speed is reached, drive upwards with full speed
                        //if endpos is reached, stop
                        #ifdef _debug
                                printf("Upwards full speed\n");
                        #endif
                        if(Stepper.up()){
                            #ifdef _debug
                                printf("In upper pos\n");
                            #endif
                            #ifdef _playerEnabled
                                Player.play(DFRobotDFPlayerMini::inUpperPos);
                            #endif
                        }
                    }
                }
                else{
                    //delay to deactivate the solenoid after stopping to drive upwards
                    if(solenoidReleaseCounter >= solenoidReleaseDelay / mainTaskPeriod){
                        Solenoid.write(false);
                        #ifdef _debug
                            printf("release Solenoid\n");
                        #endif
                    }
                    else{
                        solenoidReleaseCounter++;
                    }
                    Stepper.setVelocity(0);
                    //wird nach drücken des UserButton einmal ausgefuehrt
                        if(resetAll){
                        }
                }
                #ifdef _debug
                    printf("Motor Velocity :%f\n\n", Stepper.getVelocity());
                #endif
            break;
        }
        if(!Solenoid.read()){
            solenoidReleaseCounter = 0;
            solenoidHoldCounter = 0;
        }
        //read timer and make the main thread sleep for the remaining time span (non blocking)
        int mainTaskElapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(MainTaskTimer.elapsed_time()).count();
        //printf("\nMainTaskElapsedTime: %d\n", mainTaskElapsedTime);
        thread_sleep_for(mainTaskPeriod - mainTaskElapsedTime);
        
    }
}