/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "tune.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
DMA_HandleTypeDef hdma_tim2_ch4;

/* USER CODE BEGIN PV */
char bi_direction = 0;
char coasting = 0;

int stop_time = 0;


int current_state= 0;
uint16_t Current_GPIO_Pin;

int filter_level_up = 4;
int filter_level_down = 4;


#define TEMP30_CAL_VALUE            ((uint16_t*)((uint32_t)0x1FFFF7B8))
#define TEMP110_CAL_VALUE           ((uint16_t*)((uint32_t)0x1FFFF7C2))

int temp110cal;
int temp30cal;
int smoothedinput = 0;

const int numReadings = 100;     // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;
int readings[100];
int tempraw = 0;
int temp_degrees = 0;


int max_change;
int offset;

int ticks = 0;
int filter_level = 5;
int toggled = 0;
int running = 0;
int advance = 0;
int advancedivisor = 3;
int blanktime;
int START_ARR=800;
int rising = 1;
int count = 0;
				      // start middle phase 5
const int pwmSin[] = {128, 132, 136, 140, 143, 147, 151, 155, 159, 162, 166, 170, 174, 178, 181, 185, 189, 192, 196, 200, 203, 207, 211, 214, 218, 221, 225, 228, 232, 235, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243, 242, 241, 240, 239, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243, 242, 241, 240, 239, 238, 235, 232, 228, 225, 221, 218, 214, 211, 207, 203, 200, 196, 192, 189, 185, 181, 178, 174, 170, 166, 162, 159, 155, 151, 147, 143, 140, 136, 132, 128, 124, 120, 116, 113, 109, 105, 101, 97, 94, 90, 86, 82, 78, 75, 71, 67, 64, 60, 56, 53, 49, 45, 42, 38, 35, 31, 28, 24, 21, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 21, 24, 28, 31, 35, 38, 42, 45, 49, 53, 56, 60, 64, 67, 71, 75, 78, 82, 86, 90, 94, 97, 101, 105, 109, 113, 116, 120, 124};
int sin_divider = 2;
int phase_A_position;
int phase_B_position;
int phase_C_position;
int step_delay  = 100;
char stepper_sine = 0;
int forward = 1;
int gate_drive_offset = 20;

int stuckcounter = 0;
int duty_cycle_limit;

int looptime;
int k_erpm;

char reversed_direction = 0;

int threshold_up = 20;
int threshold_down = 20;

int ADCtimer= 30;
int demagtime = 50;

int whatstepisthis = 0 ;       // for debugging
int myneutral = 1300; // for debugging
int myneutralup = 1500;

int input_buffer_size = 64;
int smallestnumber = 20000;
uint32_t dma_buffer[64];
int propulse[4] = {0,0,0,0};
int dpulse[16] =  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

int calcCRC;
int checkCRC;
int dshotcommand;

char armed = 0;
int zero_input_count = 0;

int input = 0;
int newinput =0;
char inputSet = 0;
char dshot = 0;
char proshot = 0;
char multishot = 0;
char oneshot42 = 0;
char oneshot125 = 0;
char servoPwm = 0;
int zero_crosses;


int integral = 0;
int threshold = 850;
int zcfound = 0;
int difference;
int bemfcounter;
int min_bemf_counts_up = 10;
int min_bemf_counts_down = 10;
int zcs = 0;

int adc_timer = 600;
int ROC = 1;

int lastzctime;
uint16_t thiszctime;
int upthiszctime;
int uplastzctime;
int wait_time;


int min_startup_duty = 100;

int phase = 1;

int tim2_start_arr= 616;

uint32_t last_adc_channel;

int duty_cycle= 33;   // USE EXTREME CAUTION WHEN TESTING
uint32_t ADC1ConvertedValues[1] = {0};
int bemf_rising = 1;

int step = 1;
int pot = 1000;

uint16_t commutation_interval = 30000;
int pwm = 1;
int floating =2;
int lowside = 3;
int zero_cross_offset_up= 1;  //  up and down offset adc read
int zero_cross_offset_down= 30;
int sensorless = 1;
int waitTime = 0;
int threshhold = 5;

int signaltimeout = 0;

uint8_t ubAnalogWatchdogStatus = RESET;

int bemf = 0;
int bemfb = 0;
int bemfc = 0;
int neutral = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM14_Init(void);
static void MX_ADC_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	if (x < in_min){
		x = in_min;
	}
	if (x > in_max){
		x = in_max;
	}
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

}



void getSmoothedInput() {

		total = total - readings[readIndex];
		// read from the sensor:
		readings[readIndex] = tempraw;
		// add the reading to the total:
		total = total + readings[readIndex];
		// advance to the next position in the array:
		readIndex = readIndex + 1;

		// if we're at the end of the array...
		if (readIndex >= numReadings) {
			// ...wrap around to the beginning:
			readIndex = 0;
		}

		// calculate the average:
		smoothedinput = total / numReadings;


}





void getADCs(){

	tempraw = ADC1ConvertedValues[0];
	getSmoothedInput();

}




void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	getADCs();
}
void phaseAPWM(){
	LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_13, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
}
void phaseALOW(){
	LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_13, LL_GPIO_MODE_OUTPUT); GPIOB->BSRR = GPIO_PIN_13;
	LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_8, LL_GPIO_MODE_OUTPUT); GPIOA->BRR = GPIO_PIN_8;
}
void phaseAFLOAT(){
	LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_13, LL_GPIO_MODE_OUTPUT); GPIOB->BRR = GPIO_PIN_13;
	LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_8, LL_GPIO_MODE_OUTPUT); GPIOA->BRR = GPIO_PIN_8;
}
void phaseBPWM(){
	LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_14, LL_GPIO_MODE_ALTERNATE); // low
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);  // high
}
void phaseBLOW(){
	LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_14, LL_GPIO_MODE_OUTPUT); GPIOB->BSRR = GPIO_PIN_14;
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_9, LL_GPIO_MODE_OUTPUT); GPIOA->BRR = GPIO_PIN_9;
}
void phaseBFLOAT(){
	LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_14, LL_GPIO_MODE_OUTPUT); GPIOB->BRR = GPIO_PIN_14;
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_9, LL_GPIO_MODE_OUTPUT); GPIOA->BRR = GPIO_PIN_9;
}
void phaseCPWM(){
	LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_15, LL_GPIO_MODE_ALTERNATE);
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);
}
void phaseCLOW(){
	LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_15, LL_GPIO_MODE_OUTPUT); GPIOB->BSRR = GPIO_PIN_15;
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_10, LL_GPIO_MODE_OUTPUT); GPIOA->BRR = GPIO_PIN_10;
}
void phaseCFLOAT(){
	LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_15, LL_GPIO_MODE_OUTPUT); GPIOB->BRR = GPIO_PIN_15;
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_10, LL_GPIO_MODE_OUTPUT); GPIOA->BRR = GPIO_PIN_10;
}
void phaseA(int newPhase){

		if (newPhase==1){
		LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_13, LL_GPIO_MODE_ALTERNATE);
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
		}

		if (newPhase==2){
		LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_13, LL_GPIO_MODE_OUTPUT); GPIOB->BRR = GPIO_PIN_13;
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_8, LL_GPIO_MODE_OUTPUT); GPIOA->BRR = GPIO_PIN_8;
		}


		if (newPhase==3){
		LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_13, LL_GPIO_MODE_OUTPUT); GPIOB->BSRR = GPIO_PIN_13;
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_8, LL_GPIO_MODE_OUTPUT); GPIOA->BRR = GPIO_PIN_8;
		}
}

void phaseB(int newPhase){
	if (newPhase==1){
	LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_14, LL_GPIO_MODE_ALTERNATE); // low
	LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);  // high
	}

	if (newPhase==2){
	LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_14, LL_GPIO_MODE_OUTPUT); GPIOB->BRR = GPIO_PIN_14;
	LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_9, LL_GPIO_MODE_OUTPUT); GPIOA->BRR = GPIO_PIN_9;
	}


	if (newPhase==3){          // low mosfet on
	LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_14, LL_GPIO_MODE_OUTPUT); GPIOB->BSRR = GPIO_PIN_14;
	LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_9, LL_GPIO_MODE_OUTPUT); GPIOA->BRR = GPIO_PIN_9;
	}

}

void phaseC(int newPhase){
	if (newPhase==1){
	LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_15, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);
	}

	if (newPhase==2){
	LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_15, LL_GPIO_MODE_OUTPUT); GPIOB->BRR = GPIO_PIN_15;
	LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_10, LL_GPIO_MODE_OUTPUT); GPIOA->BRR = GPIO_PIN_10;
	}


	if (newPhase==3){
	LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_15, LL_GPIO_MODE_OUTPUT); GPIOB->BSRR = GPIO_PIN_15;
	LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_10, LL_GPIO_MODE_OUTPUT); GPIOA->BRR = GPIO_PIN_10;
	}

}

void allOff(){
	phaseA(floating);
	phaseB(floating);
	phaseC(floating);
}
//void  comStep (int newStep){
////TIM14->CNT = 0;
//
//	if(coasting){
//	allOff();
//	}else{
//
//	if (newStep==1){			//A-B
//		phaseA(pwm);
//		phaseB(lowside);
//		phaseC(floating);
//
//		phase_A_position = 88;
//		phase_B_position = 208;
//		phase_C_position = 328;
//
//		//TIM1->CCR4 = ADCtimer;
//	}
//
//	if (newStep==2){			// C-B
//		phaseA(floating);
//		phaseB(lowside);
//		phaseC(pwm);
//	//	TIM1->CCR4 = ADCtimer;
//	}
//
//
//	if (newStep==3){		// C-A
//		phaseA(lowside);
//		phaseB(floating);
//		phaseC(pwm);
//	//	TIM1->CCR4 = ADCtimer;
//	}
//
//	if (newStep==4){    // B-A
//		phaseA(lowside);
//		phaseB(pwm);
//		phaseC(floating);
//	//	TIM1->CCR4 = ADCtimer;
//	}
//
//	if (newStep==5){          // B-C
//		phaseA(floating);
//		phaseB(pwm);
//		phaseC(lowside);
//	//	TIM1->CCR4 = ADCtimer;
//	}
//
//	if (newStep==6){       // A-C
//		phaseA(pwm);
//		phaseB(floating);
//		phaseC(lowside);
//	//	TIM1->CCR4 = ADCtimer;
//	}
//	}
////stop_time = TIM14->CNT;
//
//}


void  comStep (int newStep){
//TIM14->CNT = 0;
switch(newStep)
{

        case 1:			//A-B
        	phaseAPWM();
        	phaseBLOW();
        	phaseCFLOAT();
        	break;


        case 2:		// C-B
        	phaseAFLOAT();
        	phaseBLOW();
        	phaseCPWM();
        	break;



        case 3:	// C-A
        	phaseALOW();
        	phaseBFLOAT();
        	phaseCPWM();
        	break;


        case 4:// B-A
        	phaseALOW();
        	phaseBPWM();
        	phaseCFLOAT();
        	break;


        case 5:    // B-C
        	phaseAFLOAT();
        	phaseBPWM();
        	phaseCLOW();
        	break;


        case 6:      // A-C
        	phaseAPWM();
        	phaseBFLOAT();
        	phaseCLOW();
        	break;
	}

//stop_time = TIM14->CNT;

}

void fullBrake(){                     // full braking shorting all low sides
	phaseA(lowside);
	phaseB(lowside);
	phaseC(lowside);
}


void allpwm(){                        // for stepper_sine
	phaseA(pwm);
	phaseB(pwm);
	phaseC(pwm);
}

void detectInput(){
	smallestnumber = 20000;
	dshot = 0;
	proshot = 0;
	multishot = 0;
	oneshot42 = 0;
	oneshot125 = 0;
	servoPwm = 0;
	int lastnumber = dma_buffer[0];
	for ( int j = 1 ; j < input_buffer_size; j++){

		if((dma_buffer[j] - lastnumber) < smallestnumber){ // blank space
			smallestnumber = dma_buffer[j] - lastnumber;

		}
		lastnumber = dma_buffer[j];
	}

	if ((smallestnumber > 3)&&(smallestnumber < 22)){
		dshot = 1;
	}
	if ((smallestnumber > 40 )&&(smallestnumber < 80)){
		proshot = 1;
		TIM2->PSC=1;
		TIM2->CNT = 0xffff;
	}
	if ((smallestnumber > 100 )&&(smallestnumber < 400)){
		multishot = 1;
	}
//	if ((smallestnumber > 2000 )&&(smallestnumber < 3000)){
//		oneshot42 = 1;
//	}
////	if ((smallestnumber > 3000 )&&(smallestnumber < 7000)){
////		oneshot125 = 1;
////	}
	if (smallestnumber > 500){
		servoPwm = 1;
		TIM2->PSC = 47;
		TIM2->CNT = 0xffff;
	}

	if (smallestnumber == 0){
		inputSet = 0;
	}else{

		inputSet = 1;

		HAL_Delay(50);
		//	playInputTune();
	}
	HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_4, dma_buffer , 64);
}

void computeProshotDMA(){
	int lastnumber = dma_buffer[0];
	for ( int j = 1 ; j < 9; j++){

		if(((dma_buffer[j] - lastnumber) > 1500) && ((dma_buffer[j] - lastnumber) < 50000)){ // blank space
			if ((dma_buffer[j+7] - dma_buffer[j])<10000){
				//			for ( int i = 0; i < 8; i+= 2){
				//
				//			propulse[i>>1] =map((dma_buffer[j+i+1] - dma_buffer[j+i]),48, 141, 0, 15);
				//			}

				//		for ( int i = 0; i < 8; i+= 2){
				//			 propulse[i>>1] = ((dma_buffer[j+i+1] - dma_buffer[j+i]) - 46)*11>>6;
				//		}
				for (int i = 0; i < 4; i++){

					propulse[i] = (((dma_buffer[j + i*2 +1] - dma_buffer[j + i*2])) - 23)/3;


				}

				calcCRC = ((propulse[0]^propulse[1]^propulse[2])<<3
						|(propulse[0]^propulse[1]^propulse[2])<<2
						|(propulse[0]^propulse[1]^propulse[2])<<1
						|(propulse[0]^propulse[1]^propulse[2]));
				checkCRC = (propulse[3]<<3 | propulse[3]<<2 | propulse[3]<<1 | propulse[3]);
			}


            if (calcCRC == checkCRC){
			int tocheck = ((propulse[0]<<7 | propulse[1]<<3 | propulse[2]>>1));
			if (tocheck > 2047 || tocheck < 0){
				break;
			}else{
				if(tocheck > 47){
					newinput = tocheck;
					dshotcommand = 0;
				}
				if ((tocheck <= 47)&& (tocheck > 0)){
					newinput = 0;
					dshotcommand = tocheck;    //  todo
				}
				if (tocheck == 0){
					newinput = 0;
					dshotcommand = 0;
				}
			}
            }
			break;
		}
		lastnumber = dma_buffer[j];
	}
}

void computeMSInput(){

	int lastnumber = dma_buffer[0];
	for ( int j = 1 ; j < 2; j++){

		if(((dma_buffer[j] - lastnumber) < 1500) && ((dma_buffer[j] - lastnumber) > 0)){ // blank space

			newinput = map((dma_buffer[j] - lastnumber),243,1200, 0, 2000);
			break;
		}
		lastnumber = dma_buffer[j];
	}
}

void computeOS125Input(){

	int lastnumber = dma_buffer[0];
	for ( int j = 1 ; j < 2; j++){

		if(((dma_buffer[j] - lastnumber) < 12300) && ((dma_buffer[j] - lastnumber) > 0)){ // blank space

			newinput = map((dma_buffer[j] - lastnumber),6500,12000, 0, 2000);
			break;
		}
		lastnumber = dma_buffer[j];
	}
}

void computeOS42Input(){

	int lastnumber = dma_buffer[0];
	for ( int j = 1 ; j < 2; j++){

		if(((dma_buffer[j] - lastnumber) < 4500) && ((dma_buffer[j] - lastnumber) > 0)){ // blank space

			newinput = map((dma_buffer[j] - lastnumber),2020, 4032, 0, 2000);
			break;
		}
		lastnumber = dma_buffer[j];
	}
}




void computeServoInput(){

	int lastnumber = dma_buffer[0];
	for ( int j = 1 ; j < 3; j++){

		if(((dma_buffer[j] - lastnumber) >1000 ) && ((dma_buffer[j] - lastnumber) < 2010)){ // blank space

			newinput = map((dma_buffer[j] - lastnumber), 1090, 2000, 0, 2000);
			break;
		}
		lastnumber = dma_buffer[j];
	}
}


void computeDshotDMA(){

	int lastnumber = dma_buffer[0];
	for ( int j = 1 ; j < input_buffer_size; j++){

		if(((dma_buffer[j] - lastnumber) > 50) && ((dma_buffer[j] - lastnumber) < 65000)){ // blank space

			for (int i = 0; i < 16; i++){
				dpulse[i] = ((dma_buffer[j + i*2 +1] - dma_buffer[j + i*2]) / 13) - 1;
			}

			uint8_t calcCRC = ((dpulse[0]^dpulse[4]^dpulse[8])<<3
					|(dpulse[1]^dpulse[5]^dpulse[9])<<2
					|(dpulse[2]^dpulse[6]^dpulse[10])<<1
					|(dpulse[3]^dpulse[7]^dpulse[11])
			);
			uint8_t checkCRC = (dpulse[12]<<3 | dpulse[13]<<2 | dpulse[14]<<1 | dpulse[15]);
			//

			int tocheck = (
					dpulse[0]<<10 | dpulse[1]<<9 | dpulse[2]<<8 | dpulse[3]<<7
					| dpulse[4]<<6 | dpulse[5]<<5 | dpulse[6]<<4 | dpulse[7]<<3
					| dpulse[8]<<2 | dpulse[9]<<1 | dpulse[10]);

			if(calcCRC == checkCRC){

				if (tocheck > 47){
					newinput = tocheck;
                    dshotcommand = 0;
				}
			}
			if ((tocheck <= 47)&& (tocheck > 0)){
				newinput = 0;
				dshotcommand = tocheck;    //  todo
			}
			if (tocheck == 0){
				newinput = 0;
			//	dshotcommand = 0;
			}



			break;
		}
		lastnumber = dma_buffer[j];
	}
}

void transferComplete(){
	//	TIM15->CNT = 1;
//	compit = 0;
	signaltimeout = 0;
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);


	if (inputSet == 1){
		if (dshot == 1){
			computeDshotDMA();
			HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_4, dma_buffer , 64);
		}
		if (proshot == 1){
			computeProshotDMA();
			HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_4, dma_buffer , 16);
		}

		if  (servoPwm == 1){
			computeServoInput();
			HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_4, dma_buffer , 3);

		}
		if  (multishot){
			computeMSInput();
			HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_4, dma_buffer , 3);

		}
		if  (oneshot125){
			computeOS125Input();
			HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_4, dma_buffer , 3);

		}
		if  (oneshot42){
			computeOS42Input();
			HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_4, dma_buffer , 3);

		}
		if (!armed){
			if (input < 0){
				  						input = 0;
				  					}
			 		 if (input == 0){                       // note this in input..not newinput so it will be adjusted be main loop
			 		 			zero_input_count++;
			 		 		}else{
			 		 			zero_input_count = 0;
			 		 		}



	  }

	}
}

void playStartupTune(){
	TIM1->PSC = 75;
	TIM1->CCR1 = 5;
	TIM1->CCR2 = 5;
	TIM1->CCR3 = 5;
	comStep(2);
	HAL_Delay(100);
	TIM1->PSC = 50;
	HAL_Delay(100);
	TIM1->PSC = 25;
	HAL_Delay(100);
	allOff();
	TIM1->PSC = 0;
}

void playInputTune(){
	TIM1->PSC = 100;
	TIM1->CCR1 = 5;
	TIM1->CCR2 = 5;
	TIM1->CCR3 = 5;
	comStep(2);
	HAL_Delay(100);
	TIM1->PSC = 50;
	HAL_Delay(100);
	allOff();
	TIM1->PSC = 0;
}

void getBemfState(){



	if (step == 1 || step == 4){
	   current_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
	}
    if (step == 2 || step == 5){            //        in phase two or 5 read from phase A PA2
    	current_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
	}
    if (step == 3 || step == 6){                         // phase B pa1
    	current_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);

	}

    if (rising){
    	if (current_state){
    		bemfcounter++;
    		}else{
    		bemfcounter = 0;


    	}
    }else{
    	if(!current_state){
    		bemfcounter++;
    	}else{
    		bemfcounter = 0;
    	}
    }



}




void maskPhaseInterrupts(){
	EXTI->IMR &= (0 << 1);
	EXTI->IMR &= (0 << 2);
	EXTI->IMR &= (0 << 0);
}

void changeEXTI(){
//	TIM14->CNT = 0;
	if (step == 1 || step == 4){
		EXTI->IMR |= (1 << 0);
		if (rising){
			EXTI->RTSR |= (1 << 0);
			EXTI->FTSR &= (1 << 0);
		}else{
			EXTI->FTSR |= (1 << 0);
			EXTI->RTSR &= (1 << 0);
		}
		Current_GPIO_Pin = GPIO_PIN_0;
		}
	    if (step == 2 || step == 5){            //        in phase two or 5 read from phase A PA2
	    EXTI->IMR |= (1 << 2);
	    if (rising){
	    			EXTI->RTSR |= (1 << 2);
	    			EXTI->FTSR &= (0 << 2);
	    		}else{
	    			EXTI->FTSR |= (1 << 2);
	    			EXTI->RTSR &= (0 << 2);
	    		}
	    Current_GPIO_Pin = GPIO_PIN_2;
		}
	    if (step == 3 || step == 6){                         // phase B pa1
	    EXTI->IMR |= (1 << 1);
	    if (rising){
	    			EXTI->RTSR |= (1 << 1);
	    			EXTI->FTSR &= (0 << 1);
	    		}else{
	    			EXTI->FTSR |= (1 << 1);
	    			EXTI->RTSR &= (0 << 1);
	    		}
	    Current_GPIO_Pin = GPIO_PIN_1;
		}
//	    stop_time = TIM14->CNT;
}


void calculateOffset(){                      // max offset same as blank time
if (zero_crosses > 100){
	max_change = commutation_interval >> 3;
	if (thiszctime < (commutation_interval - max_change)){
offset = commutation_interval - thiszctime - max_change;
//waitTime = waitTime + offset;
	}
	else if (thiszctime > (commutation_interval + max_change)){
//offset = thiszctime - commutation_interval + max_change;
//waitTime = waitTime - offset;
if (waitTime < 1){
	waitTime = 1;
}
	}else{
		offset = 0;
	}
}
}


void commutate(){
	TIM14->CNT = 0;
	if (forward == 1){
		step++;
		if (step > 6) {
			step = 1;
		}
		if (step == 1 || step == 3 || step == 5) {
			rising = 1;                                // is back emf rising or falling
		}
		if (step == 2 || step == 4 || step == 6) {
			rising = 0;
		}
	}
	if (forward == 0){
		step--;
		if (step < 1) {
			step = 6;
		}
		if (step == 1 || step == 3 || step == 5) {
			rising = 0;
		}
		if (step == 2 || step == 4 || step == 6) {
			rising = 1;
		}
	}
	comStep(step);
//    changeEXTI();                done at the end of exti routine
	bemfcounter = 0;
	zcfound = 0;
	stop_time = TIM14->CNT;
}

void interruptRoutine(){
	thiszctime = TIM3->CNT;
//	TIM14->CNT = 0;
	if (commutation_interval > 200){
stuckcounter++;
if (stuckcounter > 100){
	maskPhaseInterrupts();
	zero_crosses = 0;
	return;
}




	//		GPIOA->BSRR = GPIO_PIN_15;
			if (rising){

				for (int i = 0; i < filter_level; i++){
				if (HAL_GPIO_ReadPin(GPIOA, Current_GPIO_Pin) == 0){
											return;
				}
				}
			}else{
				for (int i = 0; i < filter_level; i++){
				if (HAL_GPIO_ReadPin(GPIOA, Current_GPIO_Pin) == 1){
							return;
					}
				}
			}
	}
	//		while (TIM3->CNT - thiszctime < filter_delay){
	//
	//		}

			TIM3->CNT = 0;
//			zctimeout = 0;
			maskPhaseInterrupts();
//calculateOffset();


	//		stop_time = TIM14->CNT;
						while (TIM3->CNT  < waitTime){
									}
						      //      TIM1->CNT = duty_cycle;

									commutate();
									while (TIM3->CNT  < waitTime + blanktime){
									}
//                                 if((thiszctime < (commutation_interval + max_change)) && (thiszctime > (commutation_interval - max_change))){
//                                	 commutation_interval = thiszctime ;
//                                 }else{
							             // TEST!   divide by two when tracking up down time independant

	//										}
									commutation_interval = ((4 *commutation_interval) + thiszctime) / 5;
									                                             advance = commutation_interval / advancedivisor;
																					waitTime = commutation_interval /2  - advance;
																					blanktime = commutation_interval / 8;
												changeEXTI();
												zero_crosses++;
									return;


}
void advanceincrement(){
//	  if (stepper_sine){

	phase_A_position ++;
//	if (phase_A_position >= 29 && phase_A_position <= 88){                // this is wrong changr this!!!
//		step = 6;
//	}
//	if (phase_A_position >= 99 && phase_A_position <= 158){
//			step = 1;
//		}
//	if (phase_A_position >= 159 && phase_A_position <= 218){
//			step = 2;
//		}
//	if (phase_A_position >= 219 && phase_A_position <= 278){
//				step = 3;
//			}
//	 if (phase_A_position >= 279 && phase_A_position <= 328){
//			step = 4;
//		}
//	if (phase_A_position >= 329 || phase_A_position <= 28){
//			step = 5;
//		}




	     if (phase_A_position > 359){
		   phase_A_position = 0 ;
	     }

		    phase_B_position ++;
		     if (phase_B_position > 359){
			phase_B_position = 0 ;
		}
		    phase_C_position ++;
		     if (phase_C_position > 359){
			phase_C_position = 0 ;
		}


		    TIM1->CCR1 = (pwmSin[phase_A_position]/sin_divider)+gate_drive_offset;												// set duty cycle to 50 out of 768 to start.
		    TIM1->CCR2 = (pwmSin[phase_B_position]/sin_divider)+gate_drive_offset;
		    TIM1->CCR3 = (pwmSin[phase_C_position]/sin_divider)+gate_drive_offset;



	//	  }
//		  else{
		//	 TIM1->CCR1 = 0;												// set duty cycle to 50 out of 768 to start.
	//		 TIM1->CCR2 = 0;
	//		 TIM1->CCR3 = 0;
	//	  }
}





void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

//			if (htim->Instance==TIM14)
//
//
//			{
//				if (stepper_sine){
//
//			    TIM14->ARR = step_delay;
//			//	GPIOF->BSRR = GPIO_PIN_0;
//				advanceincrement();
//				}
//			}else{

//			}

}






int getAbsDif(int number1, int number2){
	int result = number1 - number2;
	if (result < 0) {
	    result = -result;
	}
	return result;
}


//void startMotor(){
//
//	        TIM1->CCR1 = 55;												// set duty cycle to 50 out of 499 to start.
//		    TIM1->CCR2 = 55;
//		    TIM1->CCR3 = 55;
//
//
////	GPIOF->BSRR = GPIO_PIN_6;            // uncomment to take bridge out of standby mode
////	GPIOF->BSRR = GPIO_PIN_7;
//	for (int i = 10; i > 0; i--){
//
////		TIM2->ARR = i;
////		TIM2->CNT = 0;
//
//		step++;
//		if (step > 6) {
//		step=1;
//		}
//		comStep(step);
//		HAL_Delay(i);
//
//
//	}
//	zcfound =1; //supress bemf detection for speedup
//	running = 1;
//
//
//
//	TIM2->ARR = START_ARR;
//	TIM2->CNT = 0;
//	HAL_Delay(10);
//
//
//
//	//					sensorless = 1;
//}
void startMotor() {

 //startcount++;

  //  char decaystate = slow_decay;
  //  sensorless = 0;
	if (running == 0){
	//	HAL_COMP_Stop_IT(&hcomp1);
	//	slow_decay = 1;


	commutate();
	commutation_interval = 20000;
	TIM3->CNT = 10000;
//	TIM2->CNT = 0;
//	TIM2->ARR = commutation_interval * 2;
	running = 1;
//	if (HAL_COMP_Start_IT(&hcomp1) != HAL_OK) {
//			/* Initialization Error */
//			Error_Handler();
//		}
	}
	changeEXTI();
//	slow_decay = decaystate;    // return to normal
	sensorless = 1;
//	startupcountdown =0;
//	bemf_counts = 0;

}


void zcfoundroutine(){
	thiszctime = TIM3->CNT;
	TIM3->CNT = 0;
	commutation_interval = (thiszctime + (3*commutation_interval)) / 4;
	waitTime = commutation_interval / 2;
	blanktime = commutation_interval / 4;
	while (TIM3->CNT - thiszctime < waitTime - advance){

	}
	commutate();
	while (TIM3->CNT - thiszctime < waitTime + blanktime){

	}
	//changeEXTI();
    bemfcounter = 0;


    zero_crosses++;
    if (zero_crosses == 50) {
    	changeEXTI();
    }


}



//
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//
//
//}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_IWDG_Init();
  MX_TIM14_Init();
  MX_ADC_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);             // uncomment for comp_pwm
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  HAL_Delay(10);


	if (HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
	if (HAL_ADC_Start_DMA(&hadc, (uint32_t*)ADC1ConvertedValues, 1) != HAL_OK){
		Error_Handler();

	}




	  temp110cal = *TEMP110_CAL_VALUE;
	  temp30cal = *TEMP30_CAL_VALUE;
//  if (HAL_ADC_Start_DMA(&hadc, (uint32_t*)ADC1ConvertedValues, 2) != HAL_OK)
//   return 0;

  HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_4, dma_buffer , 64);


  TIM1->CCR4 = 800;


  LL_GPIO_SetPinMode(GPIOF, GPIO_PIN_0, LL_GPIO_MODE_OUTPUT);

  //TIM1->CCR4 = 350;                // adc read timer.

  HAL_TIM_Base_Start(&htim14);
  HAL_TIM_Base_Start(&htim3);
 // HAL_TIM_Base_Start_IT(&htim14);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  maskPhaseInterrupts();
  GPIOF->BSRR = GPIO_PIN_6;            // uncomment to take bridge out of standby mode and set oc level
  GPIOF->BRR = GPIO_PIN_7;				// out of standby mode

  GPIOA->BSRR = GPIO_PIN_11;  // set overcurrent protection on.

  //TIM2->ARR = 1000;
//  last_adc_channel=ADC_CHANNEL_3;
  phase_A_position = 0;
  phase_B_position = 119;
  phase_C_position = 239;
  playStartupTune();
  running = 0;
  duty_cycle = 1;
	if(HAL_IWDG_Init(&hiwdg) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}
	allpwm();
	filter_level=5;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  looptime = TIM14->CNT;
//	  TIM14->CNT = 0;
	  count++;
	  stuckcounter = 0;
	  if (count  > 200){
	 	  if(HAL_IWDG_Refresh(&hiwdg) != HAL_OK)                   // watchdog refresh
	 	  			{
	 	  				/* Refresh Error */
	 	  				Error_Handler();
	 	  			}
	 	  count = 0;
	  }

	  if (inputSet == 0){
	 	 detectInput();
	  }
		  if (zero_input_count > 1000 && !armed){
			  armed = 1;
			  playInputTune();
		  }



		  if (bi_direction == 1 && proshot == 0){

			if ( newinput > 1100 ){
				if(reversed_direction){
				if (forward == 1){
					forward = 0 ;
				}
				}else{
				if (forward == 0){
					forward = 1 ;
				}
				}
					input = (newinput - 1050)*3;
			}

			if (newinput < 800) {
				if(reversed_direction){
					if (forward == 0){
					forward = 1;
				}
				}else{


				if (forward == 1){
					forward = 0;
				}
				}
					input = (800 - newinput) * 3;


			}


			if (newinput > 800 && newinput < 1100){
				input = 0;

			}


		}else if((proshot || dshot)&& bi_direction){
	  					if ( newinput > 1100 ){

	  					  if (!forward){
	  						forward = 1 ;
	  					  }
	  						input = (newinput - 1100) * 2 ;


	  					}if ( newinput <= 1047 ){
	  						if(forward){
	  					forward = 0;
	  						}
	  						input = (newinput - 90) * 2 ;
	  					}
	  					if ((newinput > 1047 && newinput < 1100) || newinput < 100){
	  						input = 0;
	  					}


	  				}else{
		 input = newinput;
	 }



	  if (input >= 130 && armed){
		  if (running == 0){
//			 startMotor();
			  running = 1;
		  }
		  coasting = 0;
	 //	 running = 1;

//		  if(commutation_interval < 100){
//			  duty_cycle_limit++;
//		  }else{
//			  duty_cycle_limit--;
//			  if(duty_cycle_limit< 0){
//				  duty_cycle_limit = 0;
//			  }
//		  }

	 	 duty_cycle = input / 2 - 30 - duty_cycle_limit;
	  }



	  if (input < 130){
		  if (running == 1){
		  coasting = 0;
		  }
	 	 running = 0;
	 	 duty_cycle = 0;
	 	 stepper_sine = 0;
	 	zero_crosses = 0;
	 	fullBrake();            // to track zero crosses at 0 throttle comment this out but there is only partial braking
	  }





	 //	 if (duty_cycle < 250){
//	 		 TIM1->CCR4 = 800;
	 //		 }


	 //	 if (duty_cycle > 300){
	 //		 TIM1->CCR4 = duty_cycle- 200;
	 //	 }


   if (zero_crosses < 100 && running){
	   if (duty_cycle < min_startup_duty){
	   duty_cycle = min_startup_duty;

	   }
	   if (duty_cycle > 250){
		   duty_cycle = 250;
	   }
   }

   if(commutation_interval > 20000 && running){
	   duty_cycle = input / 2 - 10 ;
	   if(commutation_interval > 30000){
	  	   duty_cycle = input / 2;
	     }
   }

   if (duty_cycle < 30 && running){
	   duty_cycle = 30;
   }

	 if (duty_cycle > 998){
		 duty_cycle = 998;
	 }


	 temp_degrees = ( ( (110 - 30)*(smoothedinput - temp30cal) ) / (temp110cal - temp30cal)) + 30;
     k_erpm = ((100000000 / (commutation_interval * 208 * 6)) * 6)/10 ;                  // 208 ns per count note: in 1000's of erpm to change to erpm divide multiply by 1000

	if (armed){
	 	 TIM1->CCR1 = duty_cycle;												// set duty cycle to 50 out of 768 to start.
	 	 TIM1->CCR2 = duty_cycle;
	 	 TIM1->CCR3 = duty_cycle;

	  }else{           // not armed

		  TIM1->CCR1 = 0;												// set duty cycle to 50 out of 768 to start.
		  TIM1->CCR2 = 0;
		  TIM1->CCR3 = 0;
	  }

	if (zero_crosses < 100 || commutation_interval > 10000){
		filter_level = 8 ;
		advancedivisor = 4;
	}
//	if (zero_crosses > 100 && commutation_interval < 10000){
//		advancedivisor = 4;
//		filter_level = 2;
//
//	}
	if (zero_crosses > 100 && zero_crosses < 300){
			filter_level = 4;
	//		advancedivisor = 4;
		}
	if(zero_crosses > 300 && duty_cycle > 180){
		filter_level = 0;
		advancedivisor = 3;
	}
if (commutation_interval < 200){
	advancedivisor = 2;
}

	if (duty_cycle < 200){
		filter_level = 8;
	}

	// 	  GPIOF->BRR = GPIO_PIN_0;




	 //	  if ( stepper_sine == 0){


///**************** old routine*********************
if (zero_crosses < 50){
	 		 getBemfState();                                      // uncomment to make run !!!!!!!!!!!!!!!




	 	  if (!zcfound){
	 		  if (rising){
	 		 if (bemfcounter > min_bemf_counts_up){
	 			 GPIOF->BSRR = GPIO_PIN_0;
	 			 zcfound = 1;
	 			 bemfcounter = 0;
	 			 zcfoundroutine();
	 		//	 break;

	 		}
	 		  }else{
	 			  if (bemfcounter > min_bemf_counts_down){
	 			  			 GPIOF->BSRR = GPIO_PIN_0;
	 			  			 zcfound = 1;
	 			  			 bemfcounter = 0;
	 			  			 zcfoundroutine();
	 			  //			 break;

	 			  		}
	 		  }
	 	  }
//	 	  */
}


	 	  if (TIM3->CNT > 50000 && running == 1){
	 		//  TIM3->CNT = commutation_interval / 2;
	 		 running = 0;
	 		  zero_crosses = 0;
	 		//  zcs = 0;
	 		  if (zero_crosses < 50){
	 		  zcfoundroutine();
	 		  }
	 	  }

	 	//  }

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14
                              |RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = ENABLE;
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC4;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_16;
	hiwdg.Init.Window = IWDG_WINDOW_DISABLE;
	hiwdg.Init.Reload = 2000;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 40;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 10;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM14 init function */
static void MX_TIM14_Init(void)
{

  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 10;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 50000;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  LL_GPIO_InitTypeDef GPIO_InitStruct;
  LL_EXTI_InitTypeDef EXTI_InitStruct;

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(GPIOF, LL_GPIO_PIN_0);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_11);

  /**/
  LL_GPIO_ResetOutputPin(GPIOF, LL_GPIO_PIN_6);

  /**/
  LL_GPIO_ResetOutputPin(GPIOF, LL_GPIO_PIN_7);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE1);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE2);

  /**/
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_0, LL_GPIO_PULL_NO);

  /**/
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_1, LL_GPIO_PULL_NO);

  /**/
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_2, LL_GPIO_PULL_NO);

  /**/
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_INPUT);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_0;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_1;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_2;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /* EXTI interrupt init*/
  NVIC_SetPriority(EXTI0_1_IRQn, 0);
  NVIC_EnableIRQ(EXTI0_1_IRQn);
  NVIC_SetPriority(EXTI2_3_IRQn, 0);
  NVIC_EnableIRQ(EXTI2_3_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
