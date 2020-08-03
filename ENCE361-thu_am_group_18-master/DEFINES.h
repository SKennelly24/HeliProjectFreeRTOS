#ifndef DEFINES_H
#define DEFINES_H

/* ****************************************************************
 * DEFINES.h
 *
 * Header file conatining all global declarations
 * for all modules.
 *
 *
 * Author: Derrick Edward, Grayson Mynott, Ryan Earwaker
 * Thu AM Group 18
 * Last modified:  29.05.2019
 *
 * ***************************************************************/

enum MODES {LANDED = 0, FIND_REF, FLYING, LANDING}; // Enumerate Heli Modes
enum ENABLES {INITIAL = -1, DISABLED, ENABLED};       // Enumerate Enable States

// ** UART ********************************************************
#define SLOWTICK_RATE_HZ    4                       // Rate at which UART transmits
#define MAX_STR_LEN         20                      // Maximum string length to transmit
#define BAUD_RATE           9600                    // Bit Rate

// ** GPIO Pin Configurations *************************************
#define RST_BUT_PERIPH      SYSCTL_PERIPH_GPIOA     // Reset Peripheral
#define RST_BUT_PORT_BASE   GPIO_PORTA_BASE         // Reset Port Base
#define RST_BUT_PIN         GPIO_PIN_6              // Reset Pin
#define RST_BUT_INT         INT_GPIOA_TM4C123       // Reset Interrupt ID

#define SW1_PERIPH          SYSCTL_PERIPH_GPIOA     // Switch 1 Peripheral
#define SW1_PORT_BASE       GPIO_PORTA_BASE         // Switch 1 Port Base
#define SW1_PIN             GPIO_PIN_7              // Switch 1 Pin

#define REF_PERIPH          SYSCTL_PERIPH_GPIOC     // Reference Peripheral
#define REF_PORT_BASE       GPIO_PORTC_BASE         // Reference Port Base
#define REF_PIN             GPIO_PIN_4              // Reference Pin

// ** PWM Configurations ******************************************
#define PWM_FIXED_FREQ      250                     // PWM Frequency
#define PWM_DUTY_MAX_M      80                      // Main Rotor Maximum Duty Cycle
#define PWM_DUTY_MIN_M      2                       // Main Rotor Minimum Duty Cycle When Flying
#define PWM_DUTY_MAX_T      70                      // Tail Rotor Minimum Duty Cycle
#define PWM_DUTY_MIN_T      2                       // Tail Rotor Minimum Duty Cycle
#define PWM_DUTY_MIN_M_L    35                      // Main Rotor Minimum Duty Cycle When Landing

// ** Quadrature Decoding *****************************************
#define PHASE_AB_PERIPH     SYSCTL_PERIPH_GPIOB     // Peripheral For Both Phases
#define PHASE_AB_PORT_BASE  GPIO_PORTB_BASE         // Port Base For Both Phases
#define PHASE_A_PIN         GPIO_PIN_0              // Phase A Pin
#define PHASE_B_PIN         GPIO_PIN_1              // Phase B Pin
#define PHASE_A_INT_PIN     GPIO_INT_PIN_0          // Interrupt On Phase A Pin
#define PHASE_B_INT_PIN     GPIO_INT_PIN_1          // Interrupt On Phase B Pin

// States for Phases
#define STATE_A             0                       // A = 0, B = 0
#define STATE_B             1                       // A = 1, B = 0
#define STATE_C             -1                      // A = 1, B = 2
#define STATE_D             -2                      // A = 0, B = 2

#define MAX_YAW             448                     // Number of slots in yaw disc * 4 - Equates to one full rotation CW
#define MIN_YAW             -448                    // Number of slots in yaw disc * 4 - Equates to one full rotation CCW

// ** D-Pad Buttons ***********************************************
enum butNames {UP = 0, DOWN, LEFT, RIGHT, NUM_BUTS};
enum butStates {RELEASED = 0, PUSHED, NO_CHANGE};

#define UP_BUT_PERIPH       SYSCTL_PERIPH_GPIOE     // Up Peripheral
#define UP_BUT_PORT_BASE    GPIO_PORTE_BASE         // Up Port Base
#define UP_BUT_PIN          GPIO_PIN_0              // Up Pin
#define UP_BUT_NORMAL       false                   // Up Inactive State (Active HIGH)

#define DOWN_BUT_PERIPH     SYSCTL_PERIPH_GPIOD     // Down Peripheral
#define DOWN_BUT_PORT_BASE  GPIO_PORTD_BASE         // Down Port Base
#define DOWN_BUT_PIN        GPIO_PIN_2              // Down Pin
#define DOWN_BUT_NORMAL     false                   // Down Inactive State (Active HIGH)

#define LEFT_BUT_PERIPH     SYSCTL_PERIPH_GPIOF     // Left Peripheral
#define LEFT_BUT_PORT_BASE  GPIO_PORTF_BASE         // Left Port Base
#define LEFT_BUT_PIN        GPIO_PIN_4              // Left Pin
#define LEFT_BUT_NORMAL     true                    // Left Inactive State (Active LOW)

#define RIGHT_BUT_PERIPH    SYSCTL_PERIPH_GPIOF     // Right Peripheral
#define RIGHT_BUT_PORT_BASE GPIO_PORTF_BASE         // Right Port Base
#define RIGHT_BUT_PIN       GPIO_PIN_0              // Right Pin
#define RIGHT_BUT_NORMAL    true                    // Right Inactive State (Active LOW)

#define NUM_BUT_POLLS 3                             // Number Of Times To Poll The Buttons (For Debouncing)

// ** Analog-Digital Conversion ***********************************
#define BUF_SIZE 20
#define SAMPLE_RATE_HZ 200

#endif /* DEFINES_H */
