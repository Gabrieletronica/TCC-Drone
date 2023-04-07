/*
 * ESC.c
 *
 *  Created on: Nov 11, 2022
 *      Author: gabri
 */
#include "ESC.h"
extern TIM_HandleTypeDef htim2;
void calibração_esc(uint8_t pwm_max){

	uint16_t duty_T = pwm_max*0.01*htim2.Init.Period*1.0345; 			// aconselhável usar 10%
	 		     TIM2->CCR1 = duty_T;
	 		     TIM2->CCR2 = duty_T;								 	//atribui o valor de duty ao PWM
	 		     TIM2->CCR3 = duty_T;
	 		     TIM2->CCR4 = duty_T;

	HAL_Delay(4000);
	duty_T = 0.05*htim2.Init.Period*1.0345; 			// variável de controle do duty do PWM do timer 1. Varia de (5%)2000 a (10%) 4000
	 		     TIM2->CCR1 = duty_T;
	 		     TIM2->CCR2 = duty_T;								 	//atribui o valor de duty ao PWM
	 		     TIM2->CCR3 = duty_T;
	 		     TIM2->CCR4 = duty_T;
	HAL_Delay(2000);

}
