#ifndef INC_BLDC_H_
#define INC_BLDC_H_

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include "main.h"

// Define constants for PWM and ADC parameters
#define MAX_ADC_VALUE 4095       // Maximum ADC value for 12-bit resolution
#define PWM_MAX_VALUE 300        // Maximum value for PWM output
#define RPM_THRESHOLD 218         // RPM threshold for resetting

// Define step constants for BLDC motor control
#define STEP_1 1
#define STEP_2 2
#define STEP_3 3
#define STEP_4 4
#define STEP_5 5
#define STEP_6 6

// GPIO state definitions
#define LOW GPIO_PIN_RESET        // Define LOW state for GPIO pin
#define HIGH GPIO_PIN_SET         // Define HIGH state for GPIO pin

// External variable declarations
extern ADC_HandleTypeDef hadc1;   // ADC handle for ADC1
extern TIM_HandleTypeDef htim1;    // Timer handle for TIM1

// Global variables for BLDC control
extern int pwm_value;               // Current PWM value
extern int pmw_low;                 // Low PWM value for motor control

extern bool HALL_C;                 // State of Hall sensor C
extern bool HALL_B;                 // State of Hall sensor B
extern bool HALL_A;                 // State of Hall sensor A
extern uint32_t step;               // Current step for motor control
extern bool MODE;                   // Operating mode indicator
extern int rpm;                     // Current RPM of the motor
extern bool rpm_ok;                 // RPM status flag
extern uint32_t adc_value;          // ADC reading
extern uint32_t max_adc_value;      // Max value for a 12-bit ADC
extern uint32_t arr_value;          // AutoReload Register (ARR) value for PWM
extern uint32_t duty_cycle_percentage; // Duty cycle percentage for PWM control

// Function prototypes
void adjust_pwm_with_pot(void);     // Function to adjust PWM based on potentiometer input
void control_bldc(void);             // Function to control the BLDC motor

#endif /* INC_BLDC_H_ */
