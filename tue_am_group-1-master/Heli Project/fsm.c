//*****************************************************************************
//
// The code started from ADCdemo1.c - Simple interrupt driven program which samples with AIN0
// The functions associated with this are displayMeanVal(uint16_t meanVal, uint32_t count),
// initADC (void), initClock (void), ADCIntHandler(void) and initDisplay (void). These functions
// have been adjusted slightly to fit the purpose of this project.
//
// Tue am Group 1
// Creator:  P.J. Bones  UCECE
// Last modified:   8.2.2018
//
// Code we have either written ourselves or learned in the labs are the following
// updateCheckButtons(), changeScreen(void), main(void), updateCheckButtons(),
// setZeroReference(void), quadratureFSMInterrupt(void), initQuadratureGPIO(void)
// displayHeightValue(uint16_t heightVal, uint32_t count)
//
// Creators/ID: Matt Blake        - 58979250
//             Sarah Kennelly    - 76389950
//             Brendain Hennessy - 57190084
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "utils/ustdlib.h"
#include "OrbitOLED/OrbitOLEDInterface.h"
#include "buttons4.h"
#include "yaw.h"
#include "adc.h"
#include "uart.h"
#include "pwmMainGen.h"
#include "pwmTailGen.h"
#include "piController.h"
#include "reset.h"
#include "clock.h"

//*****************************************************************************
// Constants
//*****************************************************************************
#define BUF_SIZE 8
#define SYSTICK_RATE_HZ 48
#define SAMPLE_RATE_HZ 100
#define UART_RATE_HZ 4
#define YAW_DECREMENT_RATE_HZ 24
#define ALTITUDE_DECREMENT_RATE_HZ 24

#define ROW_ZERO 0
#define ROW_ONE 1
#define ROW_TWO 2
#define ROW_THREE 3
#define COLUMN_ZERO 0
#define DISPLAY_SIZE 17
#define PI_RATE_HZ 40
#define YAW_START_HZ 55

#define INITIAL_STATE 0
#define FINAL_STATE 3

typedef enum HELI_STATE {LANDED = 0, TAKEOFF = 1, HOVER = 2, LANDING = 3} HELI_STATE;

#define MAX_HEIGHT 100
//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************
//static circBuf_t g_inBuffer; // Buffer of size BUF_SIZE integers (sample values)
   // Counter for the interrupts
static uint8_t g_heliState;
static int16_t g_flagFoundZeroReference = 0;

static int32_t g_altitudeReference;
static int32_t g_yawReference;
static int32_t g_yawZeroReference;
static int16_t g_flagFoundZeroReference;

static Controller g_altitude_controller;
static Controller g_yaw_controller;


void initDisplay(void)
{
    // intialise the Orbit OLED display
    OLEDInitialise();
}

//**********************************************************************
// Sets a global state variable to be equal to an inputted state number.
// This how the FSM keeps track of if the state is take off, flying, landing
// or landed.
//**********************************************************************
void changeState(uint8_t state_num)
{
    g_heliState = state_num;
}

//**********************************************************************
// Displays Altitude, Yaw, Main rotor duty cycle and Tail rotor duty cycle
// on the OLED display.
//**********************************************************************
void display_screen(int32_t heightVal, int32_t yaw_degrees, uint32_t altitude_PWM, uint32_t yaw_PWM)
{
    char string[DISPLAY_SIZE];

    usnprintf(string, sizeof(string), "Altitude  = %3d%%", heightVal);
    OLEDStringDraw(string, COLUMN_ZERO, ROW_ZERO);

    usnprintf(string, sizeof(string), "Yaw       = %3d", yaw_degrees);
    OLEDStringDraw(string, COLUMN_ZERO, ROW_ONE);

    usnprintf(string, sizeof(string), "Main duty = %3d%%", altitude_PWM);
    OLEDStringDraw(string, COLUMN_ZERO, ROW_TWO);

    usnprintf(string, sizeof(string), "Tail duty = %3d%%", yaw_PWM);
    OLEDStringDraw(string, COLUMN_ZERO, ROW_THREE);
}

//**********************************************************************
// Function to check Switch and change the state appropriately
//**********************************************************************
void checkSwitch(void)
{
    uint8_t switchState;

    // Poll the buttons
    updateButtons();

    //Checks if the SW1 button is pushed and changes state depending on result
    switchState = checkButton(SW1);
    if (switchState == PUSHED)
    {
        changeState(1);
    }
    else if (switchState == RELEASED)
    {
        changeState(3);
    }
}

//**********************************************************************
//If the UP button is pushed checks the current altitude reference
//and updates it appropriately
//**********************************************************************
void updateUPButton(uint8_t UPbutState) {
    if (UPbutState == PUSHED)
    {
        if (g_altitudeReference < (MAX_HEIGHT - 10)) { //If not within 10% of max altitude
            g_altitudeReference = g_altitudeReference + 10;
        } else {
            g_altitudeReference = MAX_HEIGHT;
        }
    }
}

//**********************************************************************
//If the DOWN button is pushed checks the current altitude reference
//and updates it appropriately
//**********************************************************************
void updateDownButton(uint8_t DOWNbutState) {
    if (DOWNbutState == PUSHED)
    {
        // Checks lower limits of altitude if down button is pressed
        if (g_altitudeReference > 10) {
            g_altitudeReference = g_altitudeReference - 10;
        } else {
            g_altitudeReference = 0;
        }
    }
}

//**********************************************************************
//If the LEFT button is pushed checks the current yaw reference
//and updates it appropriately
//**********************************************************************
void updateLeftButton(uint8_t LEFTbutState) {
    if (LEFTbutState == PUSHED)
    {
        // Checks upper limits of yaw when left button is pressed
        if (g_yawReference >= -165) {
            g_yawReference = g_yawReference - 15;
        } else {
            g_yawReference = 345 + g_yawReference;
        }
    }
}

//**********************************************************************
//If the RIGHT button is pushed checks the current yaw reference
//and updates it appropriately
//**********************************************************************
void updateRightButton(uint8_t RIGHTbutState) {
    if (RIGHTbutState == PUSHED)
    {
       // Checks upper limits of yaw if right button is pressed
       if (g_yawReference <= 164) {
           g_yawReference = g_yawReference + 15;
       } else {
           g_yawReference = -345 + g_yawReference;
       }
   }
}
//****************************************************************************
// Function to check button presses on the tiva board and change the height and
// Yaw references appropriately
//*****************************************************************************
void updateCheckButtons(int32_t yaw_degrees, uint32_t heightVal)
{
    uint8_t butState;

    // Poll the buttons
    updateButtons();

    butState = checkButton(UP);
    updateUPButton(butState);

    butState = checkButton(DOWN);
    updateDownButton(butState);

    butState = checkButton(LEFT);
    updateLeftButton(butState);

    butState = checkButton(RIGHT);
    updateRightButton(butState);

}
//****************************************************************************
//Check if found the reference yaw, if it has then set found reference to 1 and 
//reset the integrator error and update yaw reference
//****************************************************************************
int findZeroReferenceYaw(void)
{
    int8_t foundReferenceYaw = 0;
    //Checks if the yaw zero reference has been found
    if (g_flagFoundZeroReference == 0) {
        g_flagFoundZeroReference = haveFoundZeroReferenceYaw();
    }
    else
    {
        foundReferenceYaw = 1;
        g_yawReference = getReferenceYaw();
        g_yawZeroReference = g_yawReference;
        g_yaw_controller.intergratedError = 0;
    }
    return foundReferenceYaw;
}

//****************************************************************************
//If it has reached the appropriate height and yaw it will move to state 2
//Otherwise if it still needs to rotate to reference yaw set altitude to 25
//Finally if needed to reach height then sets altitude_PWM 
//Altitude PWM is returned
//****************************************************************************
int goToStartPoisition(yaw_degrees, adc_error_signal, heightVal, rotatedToReferenceYaw)
{
    uint32_t altitude_PWM;
    //Checks if the heli has reached the altitude reference for flying and is stable
    //at the set yaw reference then changes to flying state
    if (heightVal >= g_altitudeReference && ((yaw_degrees < (g_yawZeroReference - 1)) || (yaw_degrees > (g_yawZeroReference + 1)))) {
        changeState(2);
    }
    //If the heli is not at yaw reference then keep the main rotor PWM at 25, this is so there is less friction on the heli.
    else if (rotatedToReferenceYaw == 0)
    {
        altitude_PWM = 25;
    }
    //If heli is at the yaw reference but not at altitude reference, use a control signal to set PWM for main rotor
    else
    {
        altitude_PWM = getControlSignal(&g_altitude_controller, adc_error_signal, false);
    }
    return altitude_PWM;
}

//****************************************************************************
//Gets the helicopter to find the reference yaw and get to altitude of 20%
//****************************************************************************
void take_off(void)
{
    //Setting up variables
    uint32_t  g_count;
    int32_t  yaw_degrees;
    int32_t  heightVal;
    int32_t  sample_mean_adc;
    int32_t  yaw_error_signal;
    int32_t  adc_error_signal;
    uint32_t altitude_PWM;
    uint32_t yaw_PWM;
    uint32_t prev_uart_count;
    uint32_t prev_dis_count;
    uint32_t prev_pid_count;

    int8_t foundReferenceYaw;
    uint8_t rotatedToReferenceYaw;

    g_altitudeReference = 0; //Set the reference height to 0%
    rotatedToReferenceYaw = 0; //Set the variable to 0 showing it hasn't rotated to reference Yaw
    foundReferenceYaw = 0; //Set the found reference yaw varible to 0

    //Take off while loop
    while(g_heliState == TAKEOFF) {
        //Get current height, yaw, g_count, yaw error singal and adc error signal
        sample_mean_adc = getSampleMeanADC();
        heightVal = getAltitudePercent(sample_mean_adc);
        yaw_degrees = getYawDegrees();
        g_count = getGCount();
        
        //Updating UART and display at defined frequencies
        if ((g_count - prev_uart_count) >= (SYSTICK_RATE_HZ / UART_RATE_HZ)){
            UARTDisplay(yaw_degrees, g_yawReference, heightVal, g_altitudeReference, altitude_PWM, yaw_PWM, g_heliState);
            prev_uart_count = g_count;
        }

        //Updating the OLED Display
        if ((g_count - prev_dis_count) >= (SYSTICK_RATE_HZ / DISPLAY_RATE_HZ))
        {
            display_screen(heightVal, yaw_degrees, altitude_PWM, yaw_PWM);
            prev_dis_count = g_count;
        }

        // Updating the PID controller
        if ((g_count - prev_pid_count) >= (SYSTICK_RATE_HZ / PI_RATE_HZ)){

            //Get error signals
            yaw_error_signal = getErrorSignal(g_yawZeroReference, yaw_degrees);
            adc_error_signal = g_altitudeReference - heightVal;

            //Move around until the zero reference is found
            if (foundReferenceYaw == 0) {
                yaw_PWM = YAW_START_HZ;
                altitude_PWM = 25;
                foundReferenceYaw = findZeroReferenceYaw();
                
            } else {

                //Move to the reference yaw and then up to 20% height
                yaw_PWM = getControlSignal(&g_yaw_controller, yaw_error_signal, true);
                if ((yaw_degrees < (g_yawZeroReference - 2)) || (yaw_degrees > (g_yawZeroReference + 2))) {
                    rotatedToReferenceYaw = 1;
                    g_altitudeReference = 20;
                }
                altitude_PWM = goToStartPoisition(yaw_degrees, adc_error_signal, heightVal, rotatedToReferenceYaw);
            }
            //Set the PWM for each rotor
            setTailPWM(yaw_PWM);
            setMainRotorPWM(altitude_PWM);

            prev_pid_count = g_count;
        }
    }
}

//****************************************************************************
//Polls the buttons to change the altitude and yaw references 
//Keeps the helicoptor at the desired references
//Checks the switch if it needs to change states
//****************************************************************************
void hover_loop(void)
{
    //initializes variables 
    uint32_t g_count;
    uint32_t prev_dis_count;
    uint32_t prev_uart_count;
    int32_t yaw_degrees;
    int32_t  heightVal;
    int32_t  sample_mean_adc;
    int32_t yaw_error_signal;
    int32_t adc_error_signal;
    uint32_t altitude_PWM;
    uint32_t yaw_PWM;
    uint32_t prev_pid_count;

    g_altitudeReference = 20;

    while (g_heliState == HOVER) {
        checkSwitch(); //Checks swtich and then changes the state if necessary

        //Gets the current height and yaw position
        yaw_degrees = getYawDegrees();
        sample_mean_adc = getSampleMeanADC();
        heightVal = getAltitudePercent(sample_mean_adc);

        updateCheckButtons(yaw_degrees, heightVal); //Checks the buttons and updates the references appropriately

        g_count = getGCount();

        //Updates the OLED Display
        if ((g_count - prev_dis_count) >= (SYSTICK_RATE_HZ / DISPLAY_RATE_HZ))
        {
            display_screen(heightVal, yaw_degrees, altitude_PWM, yaw_PWM);
            prev_dis_count = g_count;
        }

        //Updates the UART
        if ((g_count - prev_uart_count) >= (SYSTICK_RATE_HZ / UART_RATE_HZ)){
            UARTDisplay(yaw_degrees, g_yawReference, heightVal, g_altitudeReference, altitude_PWM, yaw_PWM, g_heliState);
            prev_uart_count = g_count;
        }

        //Updates the PID Controller
        if ((g_count - prev_pid_count) >= (SYSTICK_RATE_HZ / PI_RATE_HZ)){

            adc_error_signal = g_altitudeReference - heightVal;
            yaw_error_signal = getErrorSignal(g_yawReference, yaw_degrees);

            altitude_PWM = getControlSignal(&g_altitude_controller, adc_error_signal, false);
            yaw_PWM = getControlSignal(&g_yaw_controller, yaw_error_signal, true);

            setMainRotorPWM(altitude_PWM);
            setTailPWM(yaw_PWM);

            prev_pid_count = g_count;
        }
    }
}

//****************************************************************************
//Lands the helicoptor by rotating to the yaw reference then decrementing the 
//height and a defined frequency
//****************************************************************************
void land(void)
{
    //Initializes the variables
    uint32_t prev_uart_count;
    int32_t yaw_degrees;
    int32_t  heightVal;
    int32_t  sample_mean_adc;
    uint32_t yaw_error_signal;
    int32_t adc_error_signal;
    uint32_t altitude_PWM;
    uint32_t yaw_PWM;
    uint32_t prev_dis_count;
    uint32_t prev_pid_count;
    uint32_t g_count;
    uint32_t rotatedToReferenceYaw;

    //Sets up the landed references for yaw and altitude
    g_altitudeReference = 20;
    g_yawReference = g_yawZeroReference;
    rotatedToReferenceYaw = 0;

    while (g_heliState == LANDING)
    {
        //Get the current g_count, yaw degrees and height
        yaw_degrees = getYawDegrees();
        sample_mean_adc = getSampleMeanADC();
        heightVal = getAltitudePercent(sample_mean_adc);
        g_count = getGCount();

        //If height is zero and within a 2 degree range of yaw reference then change state
        if ((heightVal == 0) && ((yaw_degrees > (g_yawZeroReference - 2)) && (yaw_degrees < (g_yawZeroReference + 2)))) {
            g_altitudeReference = 0;
            changeState(0);
        }
        else
        {
            //Updates the control, alititude reference, uart and display at appropriate rates
            if ((g_count - prev_uart_count) >= (SYSTICK_RATE_HZ / UART_RATE_HZ)){
                UARTDisplay(yaw_degrees, g_yawReference, heightVal, g_altitudeReference, altitude_PWM, yaw_PWM, g_heliState);
                prev_uart_count = g_count;
            }

            //Updates OLED display
            if ((g_count - prev_dis_count) >= (SYSTICK_RATE_HZ / DISPLAY_RATE_HZ))
            {
                display_screen(heightVal, yaw_degrees, altitude_PWM, yaw_PWM);
                prev_dis_count = g_count;
            }

            //Used for stable landing by decrementing the reference altitude at a set rate
            if ((g_count - prev_pid_count) >= (SYSTICK_RATE_HZ / ALTITUDE_DECREMENT_RATE_HZ)){
                //If gone to the reference yaw and the altitude reference is abocve zero decrement it
                if (rotatedToReferenceYaw == 1 || g_altitudeReference > 0){
                    g_altitudeReference -= 1;
                }
            }

            //Updates PID controller
            if ((g_count - prev_pid_count) >= (SYSTICK_RATE_HZ / PI_RATE_HZ)){
                //If within certain range of yaw reference change rotated to reference yaw to 1
                if (rotatedToReferenceYaw == 0 || (yaw_degrees < (g_yawZeroReference - 1)) || (yaw_degrees > (g_yawZeroReference + 1))) {
                    rotatedToReferenceYaw = 1;
                }
                //Get error signals and set the rotor pwms
                adc_error_signal = g_altitudeReference - heightVal;
                altitude_PWM = getControlSignal(&g_altitude_controller, adc_error_signal, false);
                yaw_error_signal = getErrorSignal(g_yawZeroReference, yaw_degrees);
                yaw_PWM = getControlSignal(&g_yaw_controller, yaw_error_signal, true);
                setMainRotorPWM(altitude_PWM);
                setTailPWM(yaw_PWM);

                prev_pid_count = g_count;
            }
        }

    }
}

//****************************************************************************
//Checks the switch and updates the display and UART after the helicoptor is
//landed
//****************************************************************************
void landed(void)
{
    //Initialize appropriate variables
    uint32_t g_count;
    uint32_t prev_uart_count;
    int32_t yaw_degrees;
    int32_t  heightVal;
    int32_t  sample_mean_adc;
    int32_t altitude_PWM;
    int32_t yaw_PWM;
    uint32_t prev_dis_count;

    //Set the PWM back to zero
    altitude_PWM = 0;
    yaw_PWM = 0;
    setMainRotorPWM(altitude_PWM);
    setTailPWM(yaw_PWM);

    //Landed while loop
    while (g_heliState == LANDED) {
        checkSwitch(); //Checks switch and changes state if necessary
        yaw_degrees = getYawDegrees();
        sample_mean_adc = getSampleMeanADC();
        heightVal = getAltitudePercent(sample_mean_adc);

        //Updates the display and UART at the appropriate frequencies
        g_count = getGCount();

        //Update UART
        if ((g_count - prev_uart_count) >= (SYSTICK_RATE_HZ / UART_RATE_HZ)){
            UARTDisplay(yaw_degrees, g_yawReference, heightVal, g_altitudeReference, altitude_PWM, yaw_PWM, g_heliState);
            prev_uart_count = g_count;
        }

        //Update OLED Display
        if ((g_count - prev_dis_count) >= (SYSTICK_RATE_HZ / DISPLAY_RATE_HZ))
        {
            display_screen(heightVal, yaw_degrees, altitude_PWM, yaw_PWM);
            prev_dis_count = g_count;
        }
    }
}

//****************************************************************************
//Calls the appropriate function for the current state
//****************************************************************************
void fsm(void)
{
    //Controls the current helicopter state
    switch(g_heliState) {
        case TAKEOFF :
            take_off();
            break;
        case HOVER :
            hover_loop();
            break;
        case LANDING:
            land();
            break;
        case LANDED:
            landed();
            break;
    }
}

//****************************************************************************
//Initalizes the appropriate functions
//****************************************************************************
void initialize(void) {
    initClock();
    initADC();
    initDisplay();
    initButtons();
    initReferenceYaw();
    initQuadratureGPIO();
    initialiseMainRotorPWM();
    initialiseTailRotorPWM();
    initialiseUSB_UART();
    initResetGPIO();
}


int main(void){
    int32_t zero_reference;

    initialize();

    // hover loop gains
    init_controller(&g_altitude_controller, 130000, 2, 0, 1, 100000);
    init_controller(&g_yaw_controller, 98000, 11, 0, 1, 10000);

    g_yawReference = 0;
    g_flagFoundZeroReference = 0;

    SysCtlDelay(SysCtlClockGet() / 3);
    // Enable interrupts to the processor.
    IntMasterEnable();


    zero_reference = setZeroReference();
    g_altitudeReference = getAltitudePercent(zero_reference);
    turnOnMainPWM();
    turnOnTailPWM();

    while(1) {
        fsm();
    }
}
