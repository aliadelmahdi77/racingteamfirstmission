# BLDC Motor Control Project

## Overview
This project implements a 6-step control for a three-phase Brushless DC (BLDC) motor using an STM32 microcontroller. The motor's Electrical RPM (ERPM) is adjusted using a potentiometer, providing a flexible control mechanism. The motor is driven by six MOSFETs arranged in a three-phase H-bridge configuration, ensuring efficient current control through the motor windings.

## System Architecture
The control system comprises three half-bridge circuits to connect each motor phase to VCC or GND. This setup injects current into the motor coils, creating the magnetic fields necessary for motor operation. A motor controller manages commutation, while Hall position sensors inform the controller of the rotor's position, allowing for precise control of the half-bridge circuits.

### Hall Sensor States
The microcontroller uses interrupts to read Hall sensor values, adjusting the motor's 6-step operation accordingly.

## Connections
### MOSFET Connections
- High-side and low-side MOSFETs are connected for each motor phase.
- Driver connections are established using the IR2112 driver to manage the MOSFET gate signals.

### Motor and Potentiometer Connections
- The potentiometer (5000 ohms) is connected to ADC1, mapping its resistance to an ADC output value between 0 and 4095.

## Requirements
### ADC Potentiometer Value Mapping
- **Potentiometer Resistance to ADC Mapping**:
  - 0 Ohms (minimum) = ADC value 0
  - 5000 Ohms (maximum) = ADC value 4095

- **ERPM Mapping**:
  - Minimum speed corresponds to an ADC value of 0.
  - Maximum speed corresponds to an ADC value of 4095.

### PWM value & Duty Cycle Calculation
The PWM value is calculated as follows:
1. `duty_cycle_percentage = (adc_value * 100) / max_adc_value;`
2. `pwm_value = (duty_cycle_percentage * (arr_value + 1)) / 100;`
3. `pwm_value = 300 - pwm_value;`

### MOSFET Gate Signal Generation
- The IR2112 driver generates the gate signals for the MOSFETs:
  - Phase A: MOSFET A_H (High Side), MOSFET A_L (Low Side)
  - Phase B: MOSFET B_H (High Side), MOSFET B_L (Low Side)
  - Phase C: MOSFET C_H (High Side), MOSFET C_L (Low Side)

## Testing on Proteus
### Speed Tests
1. **Maximum Speed**: POT value of 5000 ohms
2. **Middle Speed**: POT value of 2500 ohms
3. **Minimum Speed**: POT value of 0 ohms

### Additional Information
- The RPM (Revolutions Per Minute) displayed on the motor indicates the current speed and can be adjusted as required.
- PWM signals and Hall sensor signals can be monitored as voltage versus time graphs.

