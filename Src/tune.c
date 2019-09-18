#include "tune.h"

extern void comStep();
extern void allOff();


//void playStartupTune(){
//	TIM1->PSC = 75;
//	TIM1->CCR1 = 5;
//	TIM1->CCR2 = 5;
//	TIM1->CCR3 = 5;
//	comStep(2);
//	HAL_Delay(100);
//	TIM1->PSC = 50;
//	HAL_Delay(100);
//	TIM1->PSC = 25;
//	HAL_Delay(100);
//	allOff();
//	TIM1->PSC = 0;
//}
//
//void playInputTune(){
//	TIM1->PSC = 100;
//	TIM1->CCR1 = 5;
//	TIM1->CCR2 = 5;
//	TIM1->CCR3 = 5;
//	comStep(2);
//	HAL_Delay(100);
//	TIM1->PSC = 50;
//	HAL_Delay(100);
//	allOff();
//	TIM1->PSC = 0;
//}
