/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program subroutine
	* Author						 : Ashraf Suyyagh
	* Version            : 1.0.0
	* Date							 : January 14th, 2016
  ******************************************************************************
  */
	
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "supporting_functions.h"
#include "lis3dsh.h"
#include "math.h"
/* External variables ---------------------------------------------------------*/
extern int accelerometerInt;
extern int tick;

/* Private variables ---------------------------------------------------------*/
const double PI = 3.1415926535;
LIS3DSH_InitTypeDef ACCEL_INIT;
LIS3DSH_DRYInterruptConfigTypeDef ACCEL_INT_INIT;
SPI_HandleTypeDef* SPIInit;

//	     A0	
//	  	____
//F5	 |    |  B1
//	 	 |_G6_|
//E4	 |    |  C2
//	 	 |____|
//       D3
//Select bit 1 - 2 - 3 - 4

//Constants to specify the segements for each number to display
#define SEGMENT_NINE 		(uint16_t)(GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6)
#define SEGMENT_EIGHT 	(uint16_t)(GPIO_PIN_3|GPIO_PIN_1|GPIO_PIN_0|GPIO_PIN_7|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_6)
#define SEGMENT_SEVEN 	(uint16_t)(GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2)
#define SEGMENT_SIX 		(uint16_t)(GPIO_PIN_0|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_3)
#define SEGMENT_FIVE 		(uint16_t)(GPIO_PIN_0|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_2|GPIO_PIN_3)
#define SEGMENT_FOUR 		(uint16_t)(GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_5|GPIO_PIN_6)
#define SEGMENT_THREE 	(uint16_t)(GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6)
#define SEGMENT_TWO 		(uint16_t)(GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_6)
#define SEGMENT_ONE 		(uint16_t)(GPIO_PIN_1|GPIO_PIN_2)
#define SEGMENT_ZERO 		(uint16_t)(GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5)
#define SEGMENT_DEGREE	(uint16_t)(GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_5)
#define SEGMENT_DECIMAL	(unit16_t)(GPIO_PIN_7)

//Constant to define which digit to display
#define SEGMENT_FIRST		(uint16_t) GPIO_PIN_0
#define SEGMENT_SECOND 	(uint16_t) GPIO_PIN_1
#define SEGMENT_THIRD		(uint16_t) GPIO_PIN_2
#define SEGMENT_FOURTH	(uint16_t) GPIO_PIN_3

#define KEYPAD_ONE			(uint16_t) (GPIO_PIN_0 | GPIO_PIN_8)
#define KEYPAD_ONE			(uint16_t) (GPIO_PIN_0 | GPIO_PIN_8)
#define KEYPAD_ONE			(uint16_t) (GPIO_PIN_0 | GPIO_PIN_8)
#define KEYPAD_ONE			(uint16_t) (GPIO_PIN_0 | GPIO_PIN_8)
#define KEYPAD_ONE			(uint16_t) (GPIO_PIN_0 | GPIO_PIN_8)

float calibrationMatrix[4][3];
float axisAcceleration[3];
float calibratedAcc[3];
int tickcount;
int maxcount = 10;
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config	(void);
void initGPIO(GPIO_TypeDef* GPIOx, uint16_t pins, uint16_t input);
void computeAngles(double Ax, double Ay,double Az, float* angles);
void storeCalibrationMatrix(void);
void applyMatrix(float Ax, float Ay, float Az);
void updateDisplay(void);

int main(void)
{	
	float systemAngles[2];
	//Initialize accelerometer structure
	ACCEL_INIT.AA_Filter_BW = LIS3DSH_AA_BW_50;
	ACCEL_INIT.Axes_Enable = LIS3DSH_XYZ_ENABLE;
	ACCEL_INIT.Continous_Update = LIS3DSH_ContinousUpdate_Disabled;
	ACCEL_INIT.Full_Scale =LIS3DSH_FULLSCALE_2;
	ACCEL_INIT.Power_Mode_Output_DataRate = LIS3DSH_DATARATE_25;
	
	//Initialize accelerometer interrupts structure
	ACCEL_INT_INIT.Dataready_Interrupt = LIS3DSH_DATA_READY_INTERRUPT_ENABLED;
	ACCEL_INT_INIT.Interrupt_signal = LIS3DSH_ACTIVE_HIGH_INTERRUPT_SIGNAL;
	ACCEL_INT_INIT.Interrupt_type = LIS3DSH_INTERRUPT_REQUEST_PULSED;
		
  /* MCU Configuration----------------------------------------------------------*/

  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
	/* Initialize GPIO E */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	
	initGPIO(GPIOE, 0xFFFF, 0);
	initGPIO(GPIOB, 0x0007, 1);//Used for the keypad
	initGPIO(GPIOB, 0xFFF0, 0);
	initGPIO(GPIOA, 0x9FFF, 0);//Used as a select signal
	initGPIO(GPIOC, 0xFFFF, 0);//Used to control segments
	initGPIO(GPIOD, 0xF000, 0);//Used to light up LEDs
	
  /* Initialize all configured peripherals */
	
	//HAL_SPI_MspInit(SPIInit);
	//Initialize Accelerometer
	LIS3DSH_Init(&ACCEL_INIT);
	//Initialize interrupts
	LIS3DSH_DataReadyInterruptConfig(&ACCEL_INT_INIT);
	//Enable external interrupt line 0
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	//Sets prioity of the external interrupt
	HAL_NVIC_SetPriority(EXTI0_IRQn,0, 1);
	
	HAL_SYSTICK_Config(168000000/1000); 
	storeCalibrationMatrix();
	tickcount = 0;
	while (1){
		if(tick)
		{
			updateDisplay();
		}
		//Reads from accelerometer when data is ready
		// also waits 1000000 ticks to make it slower/more readable
		if(accelerometerInt && tickcount < maxcount && tick) {
			tickcount++;
		}
		else if(accelerometerInt && tickcount == maxcount && tick)
		{
			//reset tickcount
			tickcount = 0;
			
			//Reads from accelerometer
			LIS3DSH_ReadACC(axisAcceleration);
			
			// calibrate the accelerometer values with pre-calculated calibration matrix
			applyMatrix(axisAcceleration[0],axisAcceleration[1],axisAcceleration[2]);
			
			// compute pitch and roll using new, calibrated values
			computeAngles(calibratedAcc[0],calibratedAcc[1],calibratedAcc[2],systemAngles);
			
			printf("x-axis: %f \n", calibratedAcc[0]);
			printf("y-axis: %f \n", calibratedAcc[1]);
			printf("z-axis: %f \n", calibratedAcc[2]);
			
			
			//computeAngles(axisAcceleration[0],axisAcceleration[1],axisAcceleration[2],systemAngles);
			
			printf("Picth: %f \n", systemAngles[1]);
			printf("Roll: %f \n", systemAngles[0]);
			//Reset interrupt flag
			accelerometerInt = 0;
			tick = 0;
		}
	}
}

//Sets ad reset each select pin as well as the pins for each segement
//Also insert delay such that digits appear as constant to the eye
void updateDisplay()
{		
		HAL_GPIO_WritePin(GPIOA,SEGMENT_FIRST,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,SEGMENT_FOUR,GPIO_PIN_SET);
		HAL_Delay(5);
		HAL_GPIO_WritePin(GPIOA,SEGMENT_FIRST,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,SEGMENT_FOUR,GPIO_PIN_RESET);
	
		HAL_GPIO_WritePin(GPIOA,SEGMENT_SECOND,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,SEGMENT_THREE,GPIO_PIN_SET);
		HAL_Delay(5);
		HAL_GPIO_WritePin(GPIOA,SEGMENT_SECOND,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,SEGMENT_THREE,GPIO_PIN_RESET);
	
		HAL_GPIO_WritePin(GPIOA,SEGMENT_THIRD,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,SEGMENT_TWO,GPIO_PIN_SET);
		HAL_Delay(5);
		HAL_GPIO_WritePin(GPIOA,SEGMENT_THIRD,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,SEGMENT_TWO,GPIO_PIN_RESET);
	
		HAL_GPIO_WritePin(GPIOA,SEGMENT_FOURTH,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,SEGMENT_ONE,GPIO_PIN_SET);
		HAL_Delay(5);
		HAL_GPIO_WritePin(GPIOA,SEGMENT_FOURTH,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,SEGMENT_ONE,GPIO_PIN_RESET);
}

void storeCalibrationMatrix()
{
	calibrationMatrix[0][0] = 0.000971848026340;	calibrationMatrix[0][1] = -0.000019145822271;	calibrationMatrix[0][2] = -0.000006527715576;
	calibrationMatrix[1][0] = -0.000020230289300;	calibrationMatrix[1][1] = 0.001000536201152;	calibrationMatrix[1][2] = -0.000011853914413;
	calibrationMatrix[2][0] = -0.000010679237958;	calibrationMatrix[2][1] = 0.000014078793958;	calibrationMatrix[2][2] = 0.000998443347062;
	calibrationMatrix[3][0] = -0.000630972996122;	calibrationMatrix[3][1] = -0.011612009697235;	calibrationMatrix[3][2] = -0.019155796902639;
}

void applyMatrix(float Ax, float Ay, float Az)
{
	calibratedAcc[0] = Ax*calibrationMatrix[0][0] + Ay*calibrationMatrix[1][0] + Az*calibrationMatrix[2][0] + calibrationMatrix[3][0];
	calibratedAcc[1] = Ax*calibrationMatrix[0][1] + Ay*calibrationMatrix[1][1] + Az*calibrationMatrix[2][1] + calibrationMatrix[3][1];
	calibratedAcc[2] = Ax*calibrationMatrix[0][2] + Ay*calibrationMatrix[1][2] + Az*calibrationMatrix[2][2] + calibrationMatrix[3][2];
}

//Compute angles
void computeAngles(double Ax, double Ay,double Az, float* angles)
{
	double pitchInRad = atan(Ax / sqrt(Ay*(Ay) + Az*(Az)));
	double rollInRad = atan(Ay / sqrt(Ax*(Ax) + Az*(Az)));
	//Convert to degrees
	pitchInRad = (pitchInRad * 360 )/ (2 * PI);
	rollInRad = (rollInRad * 360 )/ (2 * PI);
	//Return results
	angles[0] = pitchInRad;
	angles[1] = rollInRad;
}

//Initialize GPIO
void initGPIO(GPIO_TypeDef* GPIOx, uint16_t pins, uint16_t input)
{	
	GPIO_InitTypeDef GPIOInit;
	GPIOInit.Alternate = 0;	
	GPIOInit.Pin = pins;
	GPIOInit.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIOInit.Pull = GPIO_PULLDOWN;
	if(!input)
		GPIOInit.Mode = GPIO_MODE_OUTPUT_PP;
	else
		GPIOInit.Mode = GPIO_MODE_INPUT;

	//Enable GPIO	
	HAL_GPIO_Init(GPIOx, &GPIOInit);
}

/** System Clock Configuration*/
void SystemClock_Config(void){

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType 	= RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState 			 	= RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState 		= RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource 	= RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM 				= 8;
  RCC_OscInitStruct.PLL.PLLN 				= 336;
  RCC_OscInitStruct.PLL.PLLP 				= RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ 				= 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){Error_Handler(RCC_CONFIG_FAIL);};

  RCC_ClkInitStruct.ClockType 			= RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource 		= RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider 	= RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider 	= RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider 	= RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5)!= HAL_OK){Error_Handler(RCC_CONFIG_FAIL);};
	
	/*Configures SysTick to provide 1ms interval interrupts. SysTick is already 
	  configured inside HAL_Init, I don't kow why the CubeMX generates this call again*/
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	/* This function sets the source clock for the internal SysTick Timer to be the maximum,
	   in our case, HCLK is now 168MHz*/
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line){
}
#endif

