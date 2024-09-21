#include "bldc.h"
/* Function to read ADC value and adjust PWM */
void adjust_pwm_with_pot(void) {

    	// Poll for ADC conversion completion
		HAL_ADC_Start(&hadc1); // Start the ADC
		if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) { // Wait for conversion
		adc_value = HAL_ADC_GetValue(&hadc1); // Get the ADC value
		}
		HAL_ADC_Stop(&hadc1); // Stop the ADC

	    // Calculate the PWM value based on the duty cycle and timer period
		duty_cycle_percentage = (adc_value * 100) / max_adc_value; // Convert ADC to percentage
		pwm_value = (duty_cycle_percentage * (arr_value + 1)) / 100; // Convert to CCR value
		pwm_value=PWM_MAX_VALUE-pwm_value;

	    // Set the PWM output for all three channels
		__HAL_TIM_SetCompare(&htim1, C_H_Pin, pwm_value);
		__HAL_TIM_SetCompare(&htim1, B_H_Pin, pwm_value);
		__HAL_TIM_SetCompare(&htim1, A_H_Pin, pwm_value);

}

// Function to control the BLDC motor based on Hall sensor readings

void control_bldc(void) {
	// Determine the motor stepping direction
	if (MODE) {
			if (HALL_C && !HALL_B && HALL_A) { step = STEP_4; }   //HALL_C HALL_B HALL_A 1_0_1
			if (HALL_C && !HALL_B && !HALL_A) { step = STEP_5; }  //HALL_C HALL_B HALL_A 1_0_0
			if (HALL_C && HALL_B && !HALL_A) { step = STEP_6; }   //HALL_C HALL_B HALL_A 1_1_0
			if (!HALL_C && HALL_B && !HALL_A) { step = STEP_1; }  //HALL_C HALL_B HALL_A 0_1_0
			if (!HALL_C && HALL_B && HALL_A) { step = STEP_2; }   //HALL_C HALL_B HALL_A 0_1_1
			if (!HALL_C && !HALL_B && HALL_A) { step = STEP_3; }  //HALL_C HALL_B HALL_A 0_0_1
		} else {
			if (HALL_C && !HALL_B && HALL_A) { step = STEP_1; }   //HALL_C HALL_B HALL_A 1_0_1
			if (HALL_C && !HALL_B && !HALL_A) { step = STEP_2; }  //HALL_C HALL_B HALL_A 1_0_0
			if (HALL_C && HALL_B && !HALL_A) { step = STEP_3; }   //HALL_C HALL_B HALL_A 1_1_0
			if (!HALL_C && HALL_B && !HALL_A) { step = STEP_4; }  //HALL_C HALL_B HALL_A 0_1_0
			if (!HALL_C && HALL_B && HALL_A) { step = STEP_5; }   //HALL_C HALL_B HALL_A 0_1_1
			if (!HALL_C && !HALL_B && HALL_A) { step = STEP_6; }  //HALL_C HALL_B HALL_A 0_0_1
		}

	// Control the motor phase based on the determined step
    switch (step) {
        case STEP_1:
            HAL_GPIO_WritePin(A_L_GPIO_Port, A_L_Pin, LOW); // AL=L
            HAL_GPIO_WritePin(B_L_GPIO_Port, B_L_Pin, HIGH); // BL=H
            HAL_GPIO_WritePin(C_L_GPIO_Port, C_L_Pin, LOW); // CL=L

            __HAL_TIM_SetCompare(&htim1, A_H_Pin, pwm_value); // AH=H PWM
            __HAL_TIM_SetCompare(&htim1, B_H_Pin, pmw_low); // BH=L PWM
            __HAL_TIM_SetCompare(&htim1, C_H_Pin, pmw_low); // CH=L PWM
            break;
        case STEP_2:
        	HAL_GPIO_WritePin(A_L_GPIO_Port, A_L_Pin, LOW); // AL=L
            HAL_GPIO_WritePin(B_L_GPIO_Port, B_L_Pin, HIGH); // BL=H
            HAL_GPIO_WritePin(C_L_GPIO_Port, C_L_Pin, LOW); // CL=L

            __HAL_TIM_SetCompare(&htim1, A_H_Pin, pmw_low); // AH=L PWM
            __HAL_TIM_SetCompare(&htim1, B_H_Pin, pmw_low); // BH=L PWM
            __HAL_TIM_SetCompare(&htim1, C_H_Pin, pwm_value); // CH=H PWM
            break;
        case STEP_3:
        	HAL_GPIO_WritePin(A_L_GPIO_Port, A_L_Pin, HIGH); // AL=H
            HAL_GPIO_WritePin(B_L_GPIO_Port, B_L_Pin, LOW); // BL=L
            HAL_GPIO_WritePin(C_L_GPIO_Port, C_L_Pin, LOW); // CL=L

            __HAL_TIM_SetCompare(&htim1, A_H_Pin, pmw_low); // AH=L PWM
            __HAL_TIM_SetCompare(&htim1, B_H_Pin, pmw_low); // BH=L PWM
            __HAL_TIM_SetCompare(&htim1, C_H_Pin, pwm_value); // CH=H PWM
            break;
        case STEP_4:
        	 HAL_GPIO_WritePin(A_L_GPIO_Port, A_L_Pin, HIGH); // AL=H
            HAL_GPIO_WritePin(B_L_GPIO_Port, B_L_Pin, LOW); // BL=L
            HAL_GPIO_WritePin(C_L_GPIO_Port, C_L_Pin, LOW); // CL=L

            __HAL_TIM_SetCompare(&htim1, A_H_Pin, pmw_low); // AH=L PWM
            __HAL_TIM_SetCompare(&htim1, B_H_Pin, pwm_value); // BH=H PWM
            __HAL_TIM_SetCompare(&htim1, C_H_Pin, pmw_low); // CH=L PWM
            break;
        case STEP_5:
        	HAL_GPIO_WritePin(A_L_GPIO_Port, A_L_Pin, LOW); // AL=L
            HAL_GPIO_WritePin(B_L_GPIO_Port, B_L_Pin, LOW); // BL=L
            HAL_GPIO_WritePin(C_L_GPIO_Port, C_L_Pin, HIGH); // CL=H

            __HAL_TIM_SetCompare(&htim1, A_H_Pin, pmw_low); // AH=L
            __HAL_TIM_SetCompare(&htim1, B_H_Pin, pwm_value); // BH=H
            __HAL_TIM_SetCompare(&htim1, C_H_Pin, pmw_low); // CH=L PWM
            break;
        case STEP_6:
            HAL_GPIO_WritePin(A_L_GPIO_Port, A_L_Pin, LOW); // AL=L
            HAL_GPIO_WritePin(B_L_GPIO_Port, B_L_Pin, LOW); // BL=L
            HAL_GPIO_WritePin(C_L_GPIO_Port, C_L_Pin, HIGH); // CL=H

            __HAL_TIM_SetCompare(&htim1, A_H_Pin, pwm_value); // AH=H
            __HAL_TIM_SetCompare(&htim1, B_H_Pin, pmw_low); // BH=L
            __HAL_TIM_SetCompare(&htim1, C_H_Pin, pmw_low); // CH=H PWM
            rpm++;
            if (rpm == RPM_THRESHOLD) {
                rpm_ok = true;
                rpm = 0;
            }
            break;
    }
}
