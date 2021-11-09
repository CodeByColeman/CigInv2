/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mcp23017.h"

#include "lwrb.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */

//ranging space

#include "vl53l0x_class.h"
uint8_t conv_cplt=0;
uint8_t i,scaner;
VL53L0X_Dev_t sensor1, sensor2, sensor3, sensor4, s5, s6,s7, s8, s9, s10,s11,s12,s13,s14,s15,s16;



HAL_StatusTypeDef result;

// Uncomment ONE of these three lines to get
//#define LONG_RANGE
//#define HIGH_SPEED
#define HIGH_ACCURACY

void VL53L0X_Start(VL53L0X_Dev_t *lidar, uint8_t newAddr);

#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

	return ch;
}

/* round robin lowband fir filter */
#define NBSAMPLES 16

uint16_t rr_fir_filter0(uint16_t newsample) {
	static uint16_t samples[NBSAMPLES];
	static uint16_t numsample=0;
	static uint16_t slidingsum=0;
	slidingsum-= samples[numsample];
	samples[numsample]= newsample;
	slidingsum+= samples[numsample];
	numsample++;
	numsample%=NBSAMPLES;
	return slidingsum/NBSAMPLES;
}

uint16_t rr_fir_filter(uint8_t ch, uint16_t newsample) {
	if (ch>2) return 0;
	static uint16_t samples[2][NBSAMPLES]={0};
	static uint16_t numsample=0;
	static uint16_t slidingsum[2]={0};
	slidingsum[ch] -= samples[ch][numsample];
	samples[ch][numsample]= newsample;
	slidingsum[ch]+= samples[ch][numsample];
	numsample++;
	numsample%=NBSAMPLES;
	return slidingsum[ch]/NBSAMPLES;
}
















/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void GPIO_Init();

static void SensorConfig_All(void);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;


#define BUFFER_SIZE  100
volatile uint8_t data_temp_out[BUFFER_SIZE];

uint8_t data_length = 0;

volatile uint8_t data[10];
uint8_t data1[10] = {0,0,0,0,0,0,0,0,0,0};
uint8_t data_cell_RX[BUFFER_SIZE] = {0};  // len???
uint8_t data_cell_RX_stored[10] = {0,0,0,0,0,0,0,0,0,0};  // len???


char data_PC_in[BUFFER_SIZE]= {0};

int val=0;

#define DMA_RX_BUFFER_SIZE          10
uint8_t DMA_RX_Buffer[DMA_RX_BUFFER_SIZE];

/* Buffer after received data */
#define UART_BUFFER_SIZE            10
uint8_t UART_Buffer[UART_BUFFER_SIZE];
size_t Write, Read;




//abc123jkl1
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

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
	MX_USART2_UART_Init();
	MX_USART6_UART_Init();
	__HAL_UART_DISABLE_IT(&huart6, UART_IT_IDLE);
	//	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);  // Enable serial port idle interrupt

	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */
	GPIO_Init();

	// Inventory SW init.

	uint8_t numTrays = 1;
	uint8_t selsPerShelf[numTrays];// = {1}; // where each entry here is the number of selections for each shelf. So can use this for a .length
	selsPerShelf[0] = 9;

	//Alt
	uint8_t maxSelsPerTray = 16;
	int16_t tray[numTrays][maxSelsPerTray];

	uint8_t traySels[numTrays];
	traySels[0] = Tray0_Sels;
	traySels[1] = Tray1_Sels;
	traySels[2] = Tray2_Sels;
	traySels[3] = Tray3_Sels;
	traySels[4] = Tray4_Sels;
	traySels[5] = Tray5_Sels;
	traySels[6] = Tray6_Sels;
	traySels[7] = Tray7_Sels;


	int8_t config_arr[numTrays][maxSelsPerTray];  // so here, we have 2d array of empty vals where we will have space for maxSelsPerShelf number of sels.
	// 													without needing the space neccessarily. To be initialized in main.


	for (uint8_t i = 0; i<numTrays; i++){
		for (uint8_t k = 0; k<maxSelsPerTray; k++){
			if (k< selsPerShelf[i]){
				config_arr[i][k] = 0;
				//				tray[i][k] =
			}
			else{
				config_arr[i][k] = -5;
			}
		}
	}


	//alt config test
	for (uint8_t i = 0; i<=numTrays; i++){
		for (uint8_t k = 0; k<maxSelsPerTray; k++){
			if (k< traySels[i]){
				//					config_arr[i][k] = 0;
				tray[i][k] = 0;
			}
			else{
				tray[i][k] = -2;
			}
		}
	}





	////********************

	uint8_t I2C_Dev1_addr =0b11101110;// 0x77<<1;
	uint8_t I2C_Dev1_addr_2 =0b01110111;// //;
	uint8_t I2C_Dev1_addr_r = 0b11101111;//(0x77<<1)+0x01;
	uint8_t I2C_Dev2_addr = 0x76<<1;


	uint8_t d2 = 4;
	if (data[1]==4){
		HAL_Delay(1);
	}
	// lol using MCP23017 driver for PCA9548 bus sw
	MCP23017_HandleTypeDef hmcp;
	//mcp23017_init(&hmcp, &hi2c1, 0x77);

	HAL_GPIO_WritePin(ST_OUT_DI2C_EN_0_Port, ST_OUT_DI2C_EN_0_Pin, GPIO_PIN_SET); //PCA9615 channel3, (Differential mode I2c Buffer)
	HAL_GPIO_WritePin(ST_OUT_DI2C_EN_1_Port, ST_OUT_DI2C_EN_1_Pin, GPIO_PIN_SET); //PCA9615 channel3, (Differential mode I2c Buffer)
	HAL_GPIO_WritePin(ST_OUT_DI2C_EN_2_Port, ST_OUT_DI2C_EN_2_Pin, GPIO_PIN_SET); //PCA9615 channel3, (Differential mode I2c Buffer)
	HAL_GPIO_WritePin(ST_OUT_DI2C_EN_3_Port, ST_OUT_DI2C_EN_3_Pin, GPIO_PIN_SET); //PCA9615 channel3, (Differential mode I2c Buffer)
	HAL_GPIO_WritePin(ST_OUT_DI2C_EN_4_Port, ST_OUT_DI2C_EN_4_Pin, GPIO_PIN_SET); //PCA9615 channel3, (Differential mode I2c Buffer)
	HAL_GPIO_WritePin(ST_OUT_DI2C_EN_5_Port, ST_OUT_DI2C_EN_5_Pin, GPIO_PIN_SET); //PCA9615 channel3, (Differential mode I2c Buffer)
	HAL_GPIO_WritePin(ST_OUT_DI2C_EN_6_Port, ST_OUT_DI2C_EN_6_Pin, GPIO_PIN_SET); //PCA9615 channel3, (Differential mode I2c Buffer)
	HAL_GPIO_WritePin(ST_OUT_DI2C_EN_7_Port, ST_OUT_DI2C_EN_7_Pin, GPIO_PIN_SET); //PCA9615 channel3, (Differential mode I2c Buffer)


	HAL_GPIO_WritePin(SW_NREST_MAST_Port, SW_NREST_MAST_Pin, GPIO_PIN_RESET); // Rest PCA9548a
	HAL_Delay(150);
	HAL_GPIO_WritePin(SW_NREST_MAST_Port, SW_NREST_MAST_Pin, GPIO_PIN_SET);  // Power ON
	HAL_Delay(50);

	volatile HAL_StatusTypeDef status1 = HAL_I2C_IsDeviceReady(&hi2c1, I2C_Dev1_addr , 1, 1000 );
	status1 = HAL_I2C_IsDeviceReady(&hi2c1, I2C_Dev2_addr , 1, 1000 );
	status1 = HAL_I2C_IsDeviceReady(&hi2c1, I2C_Dev1_addr , 1, 1000 );  	//This the good one.
	//	status1 = HAL_I2C_IsDeviceReady(&hi2c1, I2C_Dev2_addr , 1, 1000 );
	//	status1 = HAL_I2C_IsDeviceReady(&hi2c1, I2C_Dev1_addr , 1, 1000 );


	//	uint8_t d3 = 0b11111111;
	uint8_t d3 = 0b00000001; // d3 is the selector for the MainBoardSide I2c Switch


	SensorConfig_All();

	status1 = HAL_I2C_Master_Transmit(&hi2c1, I2C_Dev1_addr ,&d3, 1, 1000);
	status1 = HAL_I2C_Master_Receive(&hi2c1, I2C_Dev1_addr ,&d2, 1, 1000);

	status1 = HAL_I2C_IsDeviceReady(&hi2c1, I2C_Dev1_addr , 1, 1000 );
	status1 = HAL_I2C_IsDeviceReady(&hi2c1, I2C_Dev1_addr , 1, 1000 );
	status1 = HAL_I2C_IsDeviceReady(&hi2c1, I2C_Dev1_addr , 1, 1000 );

	//**********************************







	//ranging more
	uint8_t sensor1_newAddr = 0x36;
	uint8_t sensor2_newAddr = 0x30;
	uint8_t sensor3_newAddr = 0x33;
	uint8_t sensor4_newAddr = 0x39;
	uint8_t s5_newAddr = 0x42;
	uint8_t s6_newAddr = 0x45;
	uint8_t s7_newAddr = 0x48;
	uint8_t s8_newAddr = 0x51;
	uint8_t s9_newAddr = 0x54;
	uint8_t s10_newAddr = 0x57;
	uint8_t s11_newAddr = 0x60;
	uint8_t s12_newAddr = 0x63;
	uint8_t s13_newAddr = 0x66;
	uint8_t s14_newAddr = 0x69;
	uint8_t s15_newAddr = 0x71;
	uint8_t s16_newAddr = 0x75;



	sensor1.I2cDevAddr = 0x29;
	sensor2.I2cDevAddr = 0x29;
	sensor3.I2cDevAddr = 0x29;
	sensor4.I2cDevAddr = 0x29;



	uint8_t idx = 0b00000000;
	//			for(idx = 0b00000000; idx<0b11111111; idx++){
	//				HAL_Delay(30);
	//				status1 = HAL_I2C_IsDeviceReady(&hi2c1, idx , 1, 1000 );
	//
	//				if ( (status1 == HAL_OK)&& ( idx != 0x77)){
	//					status1 = 69;
	//					break;
	//				}
	//				HAL_Delay(30);
	//				status1 = HAL_I2C_IsDeviceReady(&hi2c1, idx<<1 , 1, 1000 );
	//
	//				if ( (status1 == HAL_OK) && ( idx != 0x77)){
	//					status1 = 70;
	//					break;
	//				}
	//
	//			}
	//
	status1 = HAL_I2C_IsDeviceReady(&hi2c1, 0x29<<1 , 1, 1000 );


	//	VL53L0X_Start(&sensor3, 0x29);

	status1 = HAL_I2C_IsDeviceReady(&hi2c1, sensor1_newAddr<<1 , 1, 1000 );







	//
	//	for(idx = 0b00000000; idx<0b11111111; idx++){
	//		HAL_Delay(30);
	//		status1 = HAL_I2C_IsDeviceReady(&hi2c1, idx , 1, 1000 );
	//
	//		if ( (status1 == HAL_OK)&& ( idx != 0x77)){
	//			status1 = 69;
	//			break;
	//		}
	//		HAL_Delay(30);
	//		status1 = HAL_I2C_IsDeviceReady(&hi2c1, idx<<1 , 1, 1000 );
	//
	//		if ( (status1 == HAL_OK) && ( idx != 0x77)){
	//			status1 = 70;
	//			break;
	//		}
	//
	//	}







	//01001110

	//	MCP23017_HandleTypeDef hmcp;

	status1 = HAL_I2C_IsDeviceReady(&hi2c1, 0b01001110 , 1, 1000 );
	status1 = HAL_I2C_IsDeviceReady(&hi2c1, 0b10011100 , 1, 1000 );
	status1 = HAL_I2C_IsDeviceReady(&hi2c1, 0b00100111 , 1, 1000 );
	status1 = HAL_I2C_IsDeviceReady(&hi2c1, 0b01001110 , 1, 1000 );


	mcp23017_init(&hmcp, &hi2c1, MCP23017_ADDRESS_27);
	mcp23017_iodir(&hmcp, MCP23017_PORTA, MCP23017_IODIR_ALL_OUTPUT);
	mcp23017_iodir(&hmcp, MCP23017_PORTB, MCP23017_IODIR_ALL_OUTPUT);

	//	while (1) {
	//		mcp23017_read_gpio(&hmcp, MCP23017_PORTA);
	////		hmcp.gpio[MCP23017_PORTB] = hmcp.gpio[MCP23017_PORTA];


	hmcp.gpio[MCP23017_PORTB] = 0b11111111;//hmcp.gpio[MCP23017_PORTA];
	mcp23017_write_gpio(&hmcp, MCP23017_PORTB);
	//	}

	hmcp.gpio[MCP23017_PORTA] = 0b00000000;//hmcp.gpio[MCP23017_PORTA];
	mcp23017_write_gpio(&hmcp, MCP23017_PORTA);

	// RESET
	hmcp.gpio[MCP23017_PORTB] = 0b00000000;//hmcp.gpio[MCP23017_PORTA];
	mcp23017_write_gpio(&hmcp, MCP23017_PORTB);
	HAL_Delay(5000);
	// end Reset







	hmcp.gpio[MCP23017_PORTB] = 0b00000001;//hmcp.gpio[MCP23017_PORTA]   		// This releases the release line on the remote reset line of the proxim sensor.
	mcp23017_write_gpio(&hmcp, MCP23017_PORTB);
	HAL_Delay(1000);
	VL53L0X_Start(&sensor1, sensor1_newAddr);

	status1 = HAL_I2C_IsDeviceReady(&hi2c1, sensor1_newAddr , 1, 1000 );


	hmcp.gpio[MCP23017_PORTB] = 0b00000011;//hmcp.gpio[MCP23017_PORTA];
	mcp23017_write_gpio(&hmcp, MCP23017_PORTB);
	HAL_Delay(1000);
	VL53L0X_Start(&sensor2, sensor2_newAddr);

	hmcp.gpio[MCP23017_PORTB] = 0b00000111;//hmcp.gpio[MCP23017_PORTA];
	mcp23017_write_gpio(&hmcp, MCP23017_PORTB);
	HAL_Delay(1000);
	VL53L0X_Start(&sensor3, sensor3_newAddr);

	hmcp.gpio[MCP23017_PORTB] = 0b00001111;//hmcp.gpio[MCP23017_PORTA];
	mcp23017_write_gpio(&hmcp, MCP23017_PORTB);
	HAL_Delay(1000);
	VL53L0X_Start(&sensor4, sensor4_newAddr);

	hmcp.gpio[MCP23017_PORTB] = 0b00011111;//hmcp.gpio[MCP23017_PORTA];
	mcp23017_write_gpio(&hmcp, MCP23017_PORTB);
	HAL_Delay(1000);
	VL53L0X_Start(&s5, s5_newAddr);

	hmcp.gpio[MCP23017_PORTB] = 0b00111111;//hmcp.gpio[MCP23017_PORTA];
	mcp23017_write_gpio(&hmcp, MCP23017_PORTB);
	HAL_Delay(1000);
	VL53L0X_Start(&s6, s6_newAddr);

	hmcp.gpio[MCP23017_PORTB] = 0b01111111;//hmcp.gpio[MCP23017_PORTA];
	mcp23017_write_gpio(&hmcp, MCP23017_PORTB);
	HAL_Delay(1000);
	VL53L0X_Start(&s7, s7_newAddr);

	hmcp.gpio[MCP23017_PORTB] = 0b11111111;//hmcp.gpio[MCP23017_PORTA];
	mcp23017_write_gpio(&hmcp, MCP23017_PORTB);
	HAL_Delay(1000);
	VL53L0X_Start(&s8, s8_newAddr);

	/// A side

	hmcp.gpio[MCP23017_PORTA] = 0b00000001;//hmcp.gpio[MCP23017_PORTA];
	mcp23017_write_gpio(&hmcp, MCP23017_PORTA);
	HAL_Delay(1000);
	VL53L0X_Start(&s9, s9_newAddr);

	hmcp.gpio[MCP23017_PORTA] = 0b00000011;//hmcp.gpio[MCP23017_PORTA];
	mcp23017_write_gpio(&hmcp, MCP23017_PORTA);
	HAL_Delay(1000);
	VL53L0X_Start(&s10, s10_newAddr);

	hmcp.gpio[MCP23017_PORTA] = 0b00000111;//hmcp.gpio[MCP23017_PORTA];
	mcp23017_write_gpio(&hmcp, MCP23017_PORTA);
	HAL_Delay(1000);
	VL53L0X_Start(&s11, s11_newAddr);

	hmcp.gpio[MCP23017_PORTA] = 0b00001111;//hmcp.gpio[MCP23017_PORTA];
	mcp23017_write_gpio(&hmcp, MCP23017_PORTA);
	HAL_Delay(1000);
	VL53L0X_Start(&s12, s12_newAddr);

	hmcp.gpio[MCP23017_PORTA] = 0b00011111;//hmcp.gpio[MCP23017_PORTA];
	mcp23017_write_gpio(&hmcp, MCP23017_PORTA);
	HAL_Delay(1000);
	VL53L0X_Start(&s13, s13_newAddr);

	hmcp.gpio[MCP23017_PORTA] = 0b00111111;//hmcp.gpio[MCP23017_PORTA];
	mcp23017_write_gpio(&hmcp, MCP23017_PORTA);
	HAL_Delay(1000);
	VL53L0X_Start(&s14, s14_newAddr);

	hmcp.gpio[MCP23017_PORTA] = 0b01111111;//hmcp.gpio[MCP23017_PORTA];
	mcp23017_write_gpio(&hmcp, MCP23017_PORTA);
	HAL_Delay(1000);
	VL53L0X_Start(&s15, s15_newAddr);


	hmcp.gpio[MCP23017_PORTA] = 0b11111111;//hmcp.gpio[MCP23017_PORTA];
	mcp23017_write_gpio(&hmcp, MCP23017_PORTA);
	HAL_Delay(1000);
	VL53L0X_Start(&s16, s16_newAddr);




	//	/* IMPORTANT SENSOR CONFIG; UNCOMMENT THIS

	hmcp.gpio[MCP23017_PORTB] = 0b00000001;//hmcp.gpio[MCP23017_PORTA]   		// This releases the release line on the remote reset line of the proxim sensor.
	mcp23017_write_gpio(&hmcp, MCP23017_PORTB);
	HAL_Delay(1000);
	VL53L0X_Start(&sensor1, sensor1_newAddr);

	status1 = HAL_I2C_IsDeviceReady(&hi2c1, sensor1_newAddr , 1, 1000 );
	//	/*
	//	hmcp.gpio[MCP23017_PORTB] = 0b00000011;//hmcp.gpio[MCP23017_PORTA];
	//	mcp23017_write_gpio(&hmcp, MCP23017_PORTB);
	//	HAL_Delay(1000);
	//	VL53L0X_Start(&sensor2, sensor2_newAddr);
	//
	//	hmcp.gpio[MCP23017_PORTB] = 0b00000111;//hmcp.gpio[MCP23017_PORTA];
	//	mcp23017_write_gpio(&hmcp, MCP23017_PORTB);
	//	HAL_Delay(1000);
	//	VL53L0X_Start(&sensor3, sensor3_newAddr);
	//
	//		hmcp.gpio[MCP23017_PORTB] = 0b00001111;//hmcp.gpio[MCP23017_PORTA];
	//		mcp23017_write_gpio(&hmcp, MCP23017_PORTB);
	//		HAL_Delay(1000);
	//		VL53L0X_Start(&sensor4, sensor4_newAddr);

	//		hmcp.gpio[MCP23017_PORTB] = 0b00011111;//hmcp.gpio[MCP23017_PORTA];
	//		mcp23017_write_gpio(&hmcp, MCP23017_PORTB);
	//		HAL_Delay(1000);
	//		VL53L0X_Start(&s5, s5_newAddr);
	//	/*
	//	hmcp.gpio[MCP23017_PORTB] = 0b00111111;//hmcp.gpio[MCP23017_PORTA];
	//	mcp23017_write_gpio(&hmcp, MCP23017_PORTB);
	//	HAL_Delay(1000);
	//	VL53L0X_Start(&s6, s6_newAddr);
	///*
	//		hmcp.gpio[MCP23017_PORTB] = 0b01111111;//hmcp.gpio[MCP23017_PORTA];
	//		mcp23017_write_gpio(&hmcp, MCP23017_PORTB);
	//		HAL_Delay(1000);
	//		VL53L0X_Start(&s7, s7_newAddr);
	//
	hmcp.gpio[MCP23017_PORTB] = 0b11111111;//hmcp.gpio[MCP23017_PORTA];
	mcp23017_write_gpio(&hmcp, MCP23017_PORTB);
	HAL_Delay(1000);
	VL53L0X_Start(&s8, s8_newAddr);
	/*
	/// A side

	hmcp.gpio[MCP23017_PORTA] = 0b00000001;//hmcp.gpio[MCP23017_PORTA];
	mcp23017_write_gpio(&hmcp, MCP23017_PORTA);
	HAL_Delay(1000);
	VL53L0X_Start(&s9, s9_newAddr);

	hmcp.gpio[MCP23017_PORTA] = 0b00000011;//hmcp.gpio[MCP23017_PORTA];
	mcp23017_write_gpio(&hmcp, MCP23017_PORTA);
	HAL_Delay(1000);
	VL53L0X_Start(&s10, s10_newAddr);

	hmcp.gpio[MCP23017_PORTA] = 0b00000111;//hmcp.gpio[MCP23017_PORTA];
	mcp23017_write_gpio(&hmcp, MCP23017_PORTA);
	HAL_Delay(1000);
	VL53L0X_Start(&s11, s11_newAddr);

	hmcp.gpio[MCP23017_PORTA] = 0b00001111;//hmcp.gpio[MCP23017_PORTA];
	mcp23017_write_gpio(&hmcp, MCP23017_PORTA);
	HAL_Delay(1000);
	VL53L0X_Start(&s12, s12_newAddr);

	hmcp.gpio[MCP23017_PORTA] = 0b00011111;//hmcp.gpio[MCP23017_PORTA];
	mcp23017_write_gpio(&hmcp, MCP23017_PORTA);
	HAL_Delay(1000);
	VL53L0X_Start(&s13, s13_newAddr);

	hmcp.gpio[MCP23017_PORTA] = 0b00111111;//hmcp.gpio[MCP23017_PORTA];
	mcp23017_write_gpio(&hmcp, MCP23017_PORTA);
	HAL_Delay(1000);
	VL53L0X_Start(&s14, s14_newAddr);

	hmcp.gpio[MCP23017_PORTA] = 0b01111111;//hmcp.gpio[MCP23017_PORTA];
	mcp23017_write_gpio(&hmcp, MCP23017_PORTA);
	HAL_Delay(1000);
	VL53L0X_Start(&s15, s15_newAddr);


	hmcp.gpio[MCP23017_PORTA] = 0b11111111;//hmcp.gpio[MCP23017_PORTA];
	mcp23017_write_gpio(&hmcp, MCP23017_PORTA);
	HAL_Delay(1000);
	VL53L0X_Start(&s16, s16_newAddr);


	status1 = HAL_I2C_IsDeviceReady(&hi2c1, s16_newAddr , 1, 1000 );
	status1 = HAL_I2C_IsDeviceReady(&hi2c1, s15_newAddr , 1, 1000 );
	status1 = HAL_I2C_IsDeviceReady(&hi2c1, s14_newAddr , 1, 1000 );
	status1 = HAL_I2C_IsDeviceReady(&hi2c1, s13_newAddr , 1, 1000 );
	status1 = HAL_I2C_IsDeviceReady(&hi2c1, s12_newAddr , 1, 1000 );
	status1 = HAL_I2C_IsDeviceReady(&hi2c1, s11_newAddr , 1, 1000 );
	status1 = HAL_I2C_IsDeviceReady(&hi2c1, s10_newAddr , 1, 1000 );
	status1 = HAL_I2C_IsDeviceReady(&hi2c1, s9_newAddr , 1, 1000 );
	status1 = HAL_I2C_IsDeviceReady(&hi2c1, s8_newAddr , 1, 1000 );
	status1 = HAL_I2C_IsDeviceReady(&hi2c1, s7_newAddr , 1, 1000 );
	status1 = HAL_I2C_IsDeviceReady(&hi2c1, s6_newAddr , 1, 1000 );

	status1 = HAL_I2C_IsDeviceReady(&hi2c1, s5_newAddr , 1, 1000 );
	status1 = HAL_I2C_IsDeviceReady(&hi2c1, sensor4_newAddr , 1, 1000 );
	status1 = HAL_I2C_IsDeviceReady(&hi2c1, sensor1_newAddr , 1, 1000 );
	status1 = HAL_I2C_IsDeviceReady(&hi2c1, sensor2_newAddr , 1, 1000 );
	status1 = HAL_I2C_IsDeviceReady(&hi2c1, sensor1_newAddr , 1, 1000 );
	status1 = HAL_I2C_IsDeviceReady(&hi2c1, sensor3_newAddr , 1, 1000 );
	/*

//  END OF IMPORTANT SENSOR CONFIG; UNCOMMENT THIS */




	//	status1 = HAL_I2C_IsDeviceReady(&hi2c1, 0x29<<1 , 1, 1000 );
	//
	//	SetDeviceAddress(&sensor1, sensor1_newAddr);
	//	status1 = HAL_I2C_IsDeviceReady(&hi2c1, 0x29<<1 , 1, 1000 );
	//	status1 = HAL_I2C_IsDeviceReady(&hi2c1, sensor1_newAddr<<1 , 1, 1000 );
	//
	//	VL53L0X_Start(&sensor1, sensor1_newAddr);
	//	status1 = HAL_I2C_IsDeviceReady(&hi2c1, 0x29<<1 , 1, 1000 );
	//	status1 = HAL_I2C_IsDeviceReady(&hi2c1, sensor1_newAddr<<1 , 1, 1000 );



	//	/* Sensor Test Section


	uint32_t distance = 0;
	float avg_dist1 = 0;
	float avg_dist2 = 0;
	float avg_dist3 = 0;
	float avg_dist4 = 0;
	float avg_dist5 = 0;

	float alpha = 0.8;
	float beta = (1-alpha);
	int status;

	status=VL53L0X_GetDistance(&sensor1, &distance);
	status=VL53L0X_GetDistance(&sensor1, &distance);
	//	status=VL53L0X_GetDistance(&sensor2, &distance);
	//	status=VL53L0X_GetDistance(&sensor2, &distance);
	//	status=VL53L0X_GetDistance(&sensor3, &distance);// this all g
	//	status=VL53L0X_GetDistance(&sensor4, &distance);
	//	status=VL53L0X_GetDistance(&sensor4, &distance);
	status=VL53L0X_GetDistance(&s5, &distance);
	status=VL53L0X_GetDistance(&s5, &distance);
	//		status=VL53L0X_GetDistance(&sensor2, &distance);
	//		status=VL53L0X_GetDistance(&sensor2, &distance);
	//	status=VL53L0X_GetDistance(&sensor2, &distance);
	//	status=VL53L0X_GetDistance(&sensor2, &distance);
	//	status=VL53L0X_GetDistance(&sensor3, &distance);
	//	status=VL53L0X_GetDistance(&sensor3, &distance);
	//	status=VL53L0X_GetDistance(&sensor3, &distance);
	//	status=VL53L0X_GetDistance(&sensor3, &distance);
	//	status=VL53L0X_GetDistance(&sensor3, &distance);
	//	status=VL53L0X_GetDistance(&sensor3, &distance);
	//
	//	status=VL53L0X_GetDistance(&sensor3, &distance);
	//	status=VL53L0X_GetDistance(&sensor3, &distance);
	//	status=VL53L0X_GetDistance(&sensor3, &distance);
	//	status=VL53L0X_GetDistance(&sensor3, &distance);
	//	status=VL53L0X_GetDistance(&sensor3, &distance);
	//	status=VL53L0X_GetDistance(&sensor3, &distance);
	//	status=VL53L0X_GetDistance(&sensor3, &distance);
	//	status=VL53L0X_GetDistance(&sensor3, &distance);
	//	status=VL53L0X_GetDistance(&sensor3, &distance);
	//	status=VL53L0X_GetDistance(&sensor3, &distance);
	//	status=VL53L0X_GetDistance(&sensor3, &distance);
	//	status=VL53L0X_GetDistance(&sensor3, &distance);


	while(1){
		uint32_t cnt = 0;
		uint32_t cnt2 = 0;
		uint16_t s3_vals[256];
		uint16_t s2_vals[256];

		float s2_dev = 0;
		float raw_dev = 0;
		float s2_mean = 0;
		float raw_mean = 0;

		while (1){
			//			VL53L0X_GetDistance(&sensor1, &distance);
			//			avg_dist1 = avg_dist1*(alpha) + distance*(beta);
			//			VL53L0X_GetDistance(&sensor2, &distance);
			//			avg_dist2 = avg_dist2*(alpha) + distance*(beta);
			//			VL53L0X_GetDistance(&sensor3, &distance);
			//			avg_dist3 = avg_dist3*(alpha) + distance*(beta);
			//			VL53L0X_GetDistance(&sensor4, &distance);
			//			avg_dist4 = avg_dist4*(alpha) + distance*(beta);

			//
			//			VL53L0X_GetDistance(&sensor1, &distance);
			//			avg_dist1 = avg_dist1*(alpha) + distance*(beta);
			VL53L0X_GetDistance(&s8, &distance);
			avg_dist2 = avg_dist2*(alpha) + distance*(beta);
			//			VL53L0X_GetDistance(&s7, &distance);
			//			avg_dist3 = avg_dist3*(alpha) + distance*(beta);
			//			VL53L0X_GetDistance(&s8, &distance);
			//			avg_dist4 = avg_dist4*(alpha) + distance*(beta);

			s3_vals[cnt] = distance;
			s2_vals[cnt] = avg_dist2;

			HAL_Delay(5);
			cnt++;
			//			cnt = cnt % 256;
			if(cnt == 128 )
			{
				s2_mean = 0;
				raw_mean = 0;
				s2_dev = 0;
				raw_dev = 0;

				for (int iii = 0; iii<128; iii++){

					s2_mean += s2_vals[iii];
					raw_mean += s3_vals[iii];
				}
				raw_mean = raw_mean/128;
				s2_mean = s2_mean/128;

				for (int jj = 0; jj<128; jj++){
					s2_dev += (s2_mean - (s2_vals[jj])) * (s2_mean - (s2_vals[jj])) / 128;
					raw_dev += (raw_mean - (s3_vals[jj])) * (raw_mean - (s3_vals[jj])) /128;

				}
				//				s2_dev = s2_dev / 128;
				//				raw_dev = raw_dev / 128;
				//284, 263, 244

				cnt2 ++;
				cnt = 0;
			}
			//			avg_dist5 = avg_dist5*(alpha) + distance*(beta);
			HAL_Delay(1);
		}

		cnt = 0;
		HAL_Delay(100);

		//		while (cnt < 32){
		//			VL53L0X_GetDistance(&sensor1, &distance);
		//			s3_vals[cnt] = distance;
		//			HAL_Delay(5);
		//			cnt++;
		//			avg_dist1 = avg_dist1*(alpha) + distance*(beta);
		//			HAL_Delay(2);
		//
		//		}

		cnt = 0;
		HAL_Delay(100);

		while (cnt < 32){
			VL53L0X_GetDistance(&sensor2, &distance);
			s3_vals[cnt] = distance;
			HAL_Delay(5);
			cnt++;
			avg_dist2 = avg_dist2*(alpha) + distance*(beta);
			HAL_Delay(2);

		}
		cnt = 0;
		HAL_Delay(100);

		while (cnt < 32){
			VL53L0X_GetDistance(&sensor3, &distance);
			s3_vals[cnt] = distance;
			HAL_Delay(5);
			cnt++;
			avg_dist3 = avg_dist3*(alpha) + distance*(beta);
			HAL_Delay(2);

		}
		cnt = 0;
		HAL_Delay(100);
		while (cnt < 32){
			VL53L0X_GetDistance(&sensor4, &distance);
			s3_vals[cnt] = distance;
			HAL_Delay(5);
			cnt++;
			avg_dist4 = avg_dist4*(alpha) + distance*(beta);
			HAL_Delay(2);

		}

		HAL_Delay(300);
	}

	//	avg_dist = avg_dist*(alpha) + distance*(beta);
	// end ranging more


	// end Sensor test section */







	//	while(1){
	//		HAL_Delay(1000);
	//		status1 = HAL_I2C_IsDeviceReady(&hi2c1, 0x29<<1 , 1, 1000 );
	//		status1 = HAL_I2C_IsDeviceReady(&hi2c1, 0x36<<1 , 1, 1000 );
	//
	//	}

	/* Gen archive # ___




	d3 = 0b00001001;
	status1 = HAL_I2C_Master_Transmit(&hi2c1, I2C_Dev1_addr ,&d3, 1, 1000);
	status1 = HAL_I2C_Master_Transmit(&hi2c1, I2C_Dev1_addr ,&d3, 1, 1000);

	d3 = 0b00000001;
	status1 = HAL_I2C_Master_Transmit(&hi2c1, I2C_Dev1_addr ,&d3, 1, 1000);
	status1 = HAL_I2C_Master_Transmit(&hi2c1, I2C_Dev1_addr ,&d3, 1, 1000);
	status1 = HAL_I2C_Master_Transmit(&hi2c1, I2C_Dev1_addr ,&d3, 1, 1000);

	d3 = 0b00001001;

	status1 = HAL_I2C_Master_Transmit(&hi2c1, I2C_Dev1_addr ,&d3, 1, 1000);
	status1 = HAL_I2C_Master_Transmit(&hi2c1, I2C_Dev1_addr ,&d3, 1, 1000);

	status1 = HAL_I2C_Master_Receive(&hi2c1, I2C_Dev1_addr ,&d2, 1, 1000);
	d3 = 0b00000001;
	status1 = HAL_I2C_Master_Transmit(&hi2c1, I2C_Dev1_addr ,&d3, 1, 1000);
	d3 = 0b00000010;
	status1 = HAL_I2C_Master_Transmit(&hi2c1, I2C_Dev1_addr ,&d3, 1, 1000);
	d3 = 0b00000100;
	status1 = HAL_I2C_Master_Transmit(&hi2c1, I2C_Dev1_addr ,&d3, 1, 1000);
	d3 = 0b00001111;
	status1 = HAL_I2C_Master_Transmit(&hi2c1, I2C_Dev1_addr ,&d3, 1, 1000);
	//	d3 = 0b0000;
	//	status = HAL_I2C_Master_Transmit(&hi2c1, I2C_Dev1_addr ,&d3, 1, 1000);
	status1 = HAL_I2C_Master_Receive(&hi2c1, I2C_Dev1_addr ,&d2, 1, 1000);





	// oh?

	//
	//			status = HAL_I2C_IsDeviceReady(&hi2c1, I2C_Dev1_addr , 1, 1000 );
	//			status = HAL_I2C_Mem_Write(&hi2c1, I2C_Dev1_addr, 0b11111111, 1, data, 1, 10);
	//			status = HAL_I2C_Mem_Read(&hi2c1, I2C_Dev1_addr_r, 0b00001000, 1, data, 0, 100);
	//
	//			status = HAL_I2C_IsDeviceReady(&hi2c1, I2C_Dev1_addr , 1, 1000 );
	//			status = HAL_I2C_Mem_Write(&hi2c1, I2C_Dev1_addr, 0b00001000, 1, data, 1, 10);
	//			status = HAL_I2C_Mem_Read(&hi2c1, I2C_Dev1_addr_r, 0b00001000, 1, data, 1, 100);
	//
	//			status = HAL_I2C_IsDeviceReady(&hi2c1, I2C_Dev1_addr , 1, 1000 );
	//			status = HAL_I2C_Mem_Write(&hi2c1, I2C_Dev1_addr, 0b00001000, 1, data, 1, 10);
	//			status = HAL_I2C_Mem_Write(&hi2c1, I2C_Dev1_addr, 0b11111111, 1, data, 1, 10);
	//			status = HAL_I2C_IsDeviceReady(&hi2c1, I2C_Dev1_addr , 1, 1000 );

	//
	//
	//
	//
	//			status = HAL_I2C_Mem_Write(&hi2c1, I2C_Dev1_addr, 0b00001000, 1, data, 1, 10);
	//			status = HAL_I2C_Mem_Read(&hi2c1, I2C_Dev1_addr_r, 0b00001000, 1, data, 1, 100);
	//


	status1 = HAL_I2C_IsDeviceReady(&hi2c1, I2C_Dev2_addr , 1, 1000 );

	 */ //END OF Gen archive # ___







	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// uart obv
	uint8_t data[] = {'a','t','\r','\n',0,0,0,0,0,0};

	uint8_t data_PC_out[10];

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	HAL_UART_Receive_DMA(&huart6, data_PC_in, BUFFER_SIZE);
	HAL_UART_Receive_DMA(&huart2, data_cell_RX, BUFFER_SIZE);

	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */



		//				HAL_UART_Transmit(&huart2, data, 10, 1000);
		//				HAL_Delay(1);
		//				HAL_UART_Receive(&huart2, data1, 10, 1000);
		//				HAL_UART_Receive(&huart2, data1, 10, 1000);
		//
		//
		//				for(uint16_t xx = 0; xx<10; xx++){
		//					data_PC_in[xx] = 0;
		//				}
		//
		//
		//
		//		//***************************************************************** Serial terminal on uart6
		//		//***************************************************************** Uart cellular on uart2
		//
		//
		////				HAL_UART_Receive_DMA(&huart6, data_PC_in, 10, 1000);
		HAL_Delay(100);
		//
		//				HAL_UART_Transmit(&huart6, data_PC_in, 10, 1000); //echo;
		//				data_PC_in[2] ='\r';
		//				data_PC_in[3] ='\n';
		//
		//				HAL_UART_Transmit(&huart2, data_PC_in, 10, 1000); //echo;
		//				HAL_Delay(1);
		//				HAL_UART_Receive(&huart2, data1, 10, 4000);
		////				HAL_Delay();
		//				HAL_UART_Receive(&huart2, data1, 10, 1000);
		//				HAL_Delay(10);
		//
		//
		//				HAL_UART_Transmit(&huart6, data1, 10, 1000); //Return result to serial terminal.;
		//


		//		HAL_Delay(100);
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
}





/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;

	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE; // previously: UART_HWCONTROL_RTS_CTS;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);  // Enable serial port idle interrupt

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void)
{

	/* USER CODE BEGIN USART6_Init 0 */

	/* USER CODE END USART6_Init 0 */

	/* USER CODE BEGIN USART6_Init 1 */

	/* USER CODE END USART6_Init 1 */
	huart6.Instance = USART6;
	huart6.Init.BaudRate = 57600;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart6) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART6_Init 2 */
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);  // Enable serial port idle interrupt
	/* USER CODE END USART6_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	/* DMA2_Stream1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//	if(huart->Instance==USART6){  // if serial console
	//		HAL_UART_Transmit(&huart6, data_PC_in, 10, 100);
	//		if (!strcmp("at        ", *data_PC_in)){
	//			for(uint16_t xx = 3; xx<10; xx++){
	//				data_PC_in[xx] = 0 ;
	//			}
	//			data_PC_in[0] = 'a';
	//			data_PC_in[1] = 't';
	//			data_PC_in[2] = '\r';
	//			data_PC_in[3] = '\n';
	//			//			HAL_UART_Transmit(&huart2, data_PC_in, 4, 100);
	//
	//		}
	//		for(uint16_t xx = 0; xx<10; xx++){
	//			data[xx] = data_PC_in[xx];
	//		}
	//
	//	}
	//	else if(huart->Instance==USART2){  // if serial console
	//		for(uint16_t xx = 0; xx<10; xx++){
	//			data_cell_RX_stored[xx] = data_cell_RX[xx];
	//		}
	//		HAL_UART_Transmit(&huart6, data_cell_RX_stored, 10, 100); //echo to serial
	//	}
}


void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance==USART6){  // if serial console
		val = strcmp("abc123jkl1", data_PC_in);
		for(uint16_t xx = 0; xx<10; xx++){
			data[xx] = data_PC_in[xx];
		}
	}
	//	else if(huart->Instance==USART2){  // if serial console
	//		for(uint16_t xx = 0; xx<10; xx++){
	//			data_cell_RX_stored[xx] = data_cell_RX[xx];
	//		}
	//		HAL_UART_Transmit(&huart6, data_cell_RX_stored, 10, 100); //echo to serial
	//	}
}



//**ECHO MODE IRQ HANDLE
//void USER_UART_IRQHandler(UART_HandleTypeDef *huart){
//	if(USART6 == huart->Instance)                                   //Determine whether it is serial port 1
//	{
//		if(RESET != __HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE))   //Judging whether it is idle interruption
//		{
//			__HAL_UART_CLEAR_IDLEFLAG(&huart6);                     //Clear idle interrupt sign (otherwise it will continue to enter interrupt)
//
//			HAL_UART_DMAStop(&huart6);
//			data_length  = BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);
//			memset(data_temp_out,0,BUFFER_SIZE);
//			memcpy(data_temp_out,data_PC_in, data_length);
////theboyswilin
//			data_temp_out[data_length]= 13;
//			data_temp_out[data_length+1]= '\n';
//			HAL_UART_Transmit(&huart6, data_temp_out,data_length+2,100); //echo
//			//			memset(data_temp_out,0,BUFFER_SIZE);
//
//			//Zero Receiving Buffer
//
//
//			//			uint8_t test_at[10];
//			//			memset(test_at,0,10);
//			//			test_at[0]='a';
//			//			test_at[1]='t';
//			//			test_at[2]='\r';
//
//
//			//Restart to start DMA transmission of 255 bytes of data at a time
//			// logic for sending to usart2
//
//
//			//			if (!strcmp(test_at, data_temp_out)){
//			////				for(uint16_t xx = 3; xx<10; xx++){
//			////					data_temp_out[xx] = 0 ;
//			////				}
//			//				memset(data_temp_out,0,10);
//			//				data_temp_out[0] = 'a';
//			//				data_temp_out[1] = 't';
//			//				data_temp_out[2] = '\r';
//			//				//			HAL_UART_Transmit(&huart2, data_PC_in, 4, 100);
//
//
//			/* Dualie setup; I was receiving commands from a terminal window, received on UART6 and then piping them thru to UART2 where I had the cellular modul waiting for AT commands.
//			HAL_UART_Transmit(&huart2, data_temp_out,data_length,0x200);
//
//
//
//			memset(data_temp_out,0,BUFFER_SIZE);
//			memset(data_PC_in,0,BUFFER_SIZE); // alternatively, do a little temp volatile read of the buffer... not sure how dma decides to bring pointer back to front of buffer.
//			data_length = 0;
//			*/ //End of Dualie setup;
//			HAL_UART_Receive_DMA(&huart6, data_PC_in, BUFFER_SIZE);
//
//		}
//	}
//	else if(USART2 == huart->Instance)                                   //Determine whether it is serial port 1
//	{
//		if(RESET != __HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE))   //Judging whether it is idle interruption
//		{
//			__HAL_UART_CLEAR_IDLEFLAG(&huart2);                     //Clear idle interrupt sign (otherwise it will continue to enter interrupt)
//
//			HAL_UART_DMAStop(&huart2);
//			data_length  = BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
//
//			memcpy(data_temp_out, data_cell_RX, BUFFER_SIZE);
//			// This would be where we would do anything interesting with the received data.
//
//
//			//end This would be where we would do anything interesting with the received data.
//			HAL_UART_Transmit(&huart6, data_temp_out,data_length,0x200); //echo back to us.
//			memset(data_temp_out,0,data_length);
//			//Zero Receiving Buffer
//
//
//
//			//Restart to start DMA transmission of 255 bytes of data at a time
//			HAL_UART_Receive_DMA(&huart2, data_cell_RX, BUFFER_SIZE);
//			// logic for sending to usart2
//		}
//
//	}
//}


//proto for demo for irl
void USER_UART_IRQHandler(UART_HandleTypeDef *huart){
	if(USART6 == huart->Instance)                                   //Determine whether it is serial port 1
	{
		if(RESET != __HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE))   //Judging whether it is idle interruption
		{
			__HAL_UART_CLEAR_IDLEFLAG(&huart6);                     //Clear idle interrupt sign (otherwise it will continue to enter interrupt)

			HAL_UART_DMAStop(&huart6);
			data_length  = BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);

			memset(data_temp_out,0,BUFFER_SIZE);
			uint16_t index_cc = 0;
			//			memcpy(data_temp_out,data_PC_in, data_length);
			//theboyswilin

			data_temp_out[0] = 0x01;
			data_temp_out[1] = '$';
			data_temp_out[2] = 'L';
			data_temp_out[3] = 'e';
			data_temp_out[4] = 'n';
			data_temp_out[5] = ':';
			data_temp_out[6] = '0'; // needs to reflect the size of the packet
			data_temp_out[7] = '9'; // needs to reflect the size of the packet
			data_temp_out[8] = '8'; //second digit of
			data_temp_out[9] = '$';
			index_cc = 10;


			char s[50] = "LocationID:placeholder";
			char s2[25] = "numSel:";
			char s3[25] = "numShelf";
			char s4[25] = "changes:";
			char s5[5] = "s1:";
			char s6[5] = "s2:";
			char s7[5] = "s3:";
			char s8[5] = "s4:";
			char s9[5] = "s5:";
			char s10[5] = "s6:";
			char s11[5] = "s7:";
			char s12[5] = "s8:";

			char *p = s;
			uint16_t testsize = strlen(s);

			memcpy(&data_temp_out[index_cc], s, strlen(s));
			index_cc = index_cc + strlen(s);

			data_temp_out[index_cc] = '$';
			index_cc ++;

			//	numsel next
			memcpy(&data_temp_out[index_cc], s2, strlen(s2));
			index_cc = index_cc + strlen(s2);
			data_temp_out[index_cc] = '9'; // this is where the number of selections needs to be.
			// second digit
			//then '$'
			index_cc ++;
			data_temp_out[index_cc] = '$';
			index_cc ++;

			//numShelf next
			memcpy(&data_temp_out[index_cc], s3, strlen(s3));
			index_cc = index_cc + strlen(s3);
			data_temp_out[index_cc] = '8'; // this is where the number of selections needs to be.
			// second digit
			//then '$'
			index_cc ++;
			data_temp_out[index_cc] = '$';
			index_cc ++;

			//changes next
			memcpy(&data_temp_out[index_cc], s4, strlen(s4));
			index_cc = index_cc + strlen(s4);

			data_temp_out[index_cc] = '0'; // this is where the number of selections needs to be.
			// second digit
			//then '$'
			index_cc ++;
			data_temp_out[index_cc] = '$';
			index_cc ++;

			//shelf n
			memcpy(&data_temp_out[index_cc], s5, strlen(s5));
			index_cc = index_cc + strlen(s5);

			data_temp_out[index_cc] = '1'; // this is where the number of selections needs to be.
			index_cc ++;
			data_temp_out[index_cc] = ','; // this is where the number of selections needs to be.
			index_cc ++;
			data_temp_out[index_cc] = '2';
			index_cc ++;
			data_temp_out[index_cc] = ','; // this is where the number of selections needs to be.
			index_cc ++;
			data_temp_out[index_cc] = '3';
			index_cc ++;
			data_temp_out[index_cc] = ','; // this is where the number of selections needs to be.
			index_cc ++;
			data_temp_out[index_cc] = '4';
			index_cc ++;
			data_temp_out[index_cc] = ','; // this is where the number of selections needs to be.
			index_cc ++;
			data_temp_out[index_cc] = '5';
			index_cc ++;
			data_temp_out[index_cc] = ','; // this is where the number of selections needs to be.
			index_cc ++;
			data_temp_out[index_cc] = '7';
			index_cc ++;
			data_temp_out[index_cc] = '$';
			index_cc ++;

			//Indices of Len of packet reside at 6,7,8. (LSB to MSB)
			//So at this point, where we have the end length of the packet,
			// we can update the true leng.
			//Important; we'll let the length variable reflect that there are 6 additional control characters past the end of the index..
			// (these are: EOF, :, Checksum_MSB, CHECKSUM_LSB, '\r', '\n')

			data_temp_out[6] = ((index_cc+6)/100)+48;
			data_temp_out[7] = (((index_cc+6)/10)%10) +48;
			data_temp_out[8] = ((index_cc+6)%10) +48;


			uint32_t calc = 0;
			for(int ii = 0; ii< index_cc; ii++){
				calc += data_temp_out[ii];
			}

			uint32_t checksum= calc%17;
			char check_str[2];
			check_str[0] = checksum /10;
			check_str[1] = checksum %10;


			data_temp_out[index_cc] = 0x03;
			index_cc ++;
			data_temp_out[index_cc] = ':';
			index_cc ++;
			data_temp_out[index_cc] = (check_str[0])+48;
			index_cc ++;
			data_temp_out[index_cc] = (check_str[1])+48;
			index_cc ++;






			data_temp_out[index_cc]= 13;
			index_cc ++;
			data_temp_out[index_cc]= '\n';
			index_cc ++;
			HAL_UART_Transmit(&huart6, data_temp_out,index_cc+1,100); //echo
			//			memset(data_temp_out,0,BUFFER_SIZE);

			//Zero Receiving Buffer


			//			uint8_t test_at[10];
			//			memset(test_at,0,10);
			//			test_at[0]='a';
			//			test_at[1]='t';
			//			test_at[2]='\r';


			//Restart to start DMA transmission of 255 bytes of data at a time
			// logic for sending to usart2


			//			if (!strcmp(test_at, data_temp_out)){
			////				for(uint16_t xx = 3; xx<10; xx++){
			////					data_temp_out[xx] = 0 ;
			////				}
			//				memset(data_temp_out,0,10);
			//				data_temp_out[0] = 'a';
			//				data_temp_out[1] = 't';
			//				data_temp_out[2] = '\r';
			//				//			HAL_UART_Transmit(&huart2, data_PC_in, 4, 100);


			/* Dualie setup; I was receiving commands from a terminal window, received on UART6 and then piping them thru to UART2 where I had the cellular modul waiting for AT commands.
			HAL_UART_Transmit(&huart2, data_temp_out,data_length,0x200);



			memset(data_temp_out,0,BUFFER_SIZE);
			memset(data_PC_in,0,BUFFER_SIZE); // alternatively, do a little temp volatile read of the buffer... not sure how dma decides to bring pointer back to front of buffer.
			data_length = 0;
			 */ //End of Dualie setup;
			HAL_UART_Receive_DMA(&huart6, data_PC_in, BUFFER_SIZE);

		}
	}
	else if(USART2 == huart->Instance)                                   //Determine whether it is serial port 1
	{
		if(RESET != __HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE))   //Judging whether it is idle interruption
		{
			__HAL_UART_CLEAR_IDLEFLAG(&huart2);                     //Clear idle interrupt sign (otherwise it will continue to enter interrupt)

			HAL_UART_DMAStop(&huart2);
			data_length  = BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);

			memcpy(data_temp_out, data_cell_RX, BUFFER_SIZE);
			// This would be where we would do anything interesting with the received data.


			//end This would be where we would do anything interesting with the received data.
			HAL_UART_Transmit(&huart6, data_temp_out,data_length,0x200); //echo back to us.
			memset(data_temp_out,0,data_length);
			//Zero Receiving Buffer



			//Restart to start DMA transmission of 255 bytes of data at a time
			HAL_UART_Receive_DMA(&huart2, data_cell_RX, BUFFER_SIZE);
			// logic for sending to usart2
		}

	}

}


void GPIO_Init(){
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Pin = SW_NREST_MAST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(SW_NREST_MAST_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = ST_OUT_DI2C_EN_0_Pin;
	HAL_GPIO_Init(ST_OUT_DI2C_EN_0_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = ST_OUT_DI2C_EN_1_Pin;
	HAL_GPIO_Init(ST_OUT_DI2C_EN_1_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = ST_OUT_DI2C_EN_2_Pin;
	HAL_GPIO_Init(ST_OUT_DI2C_EN_2_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = ST_OUT_DI2C_EN_3_Pin;
	HAL_GPIO_Init(ST_OUT_DI2C_EN_3_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = ST_OUT_DI2C_EN_4_Pin;
	HAL_GPIO_Init(ST_OUT_DI2C_EN_4_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = ST_OUT_DI2C_EN_5_Pin;
	HAL_GPIO_Init(ST_OUT_DI2C_EN_5_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = ST_OUT_DI2C_EN_6_Pin;
	HAL_GPIO_Init(ST_OUT_DI2C_EN_6_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = ST_OUT_DI2C_EN_7_Pin;
	HAL_GPIO_Init(ST_OUT_DI2C_EN_7_Port, &GPIO_InitStruct);
}




void VL53L0X_Start(VL53L0X_Dev_t *lidar, uint8_t newAddr){
	lidar->I2cDevAddr=0x29; // ADDRESS_DEFAULT;
	lidar->comms_type=1;    // VL53L0X_COMMS_I2C
	lidar->comms_speed_khz=100;
	volatile HAL_StatusTypeDef status3;
	//	HAL_GPIO_WritePin(VL1_XSHUT_GPIO_Port, VL1_XSHUT_Pin, GPIO_PIN_RESET); //shut down the VL53L0X sensor.
	//	HAL_Delay(100); //100
	//
	//	HAL_GPIO_WritePin(VL1_XSHUT_GPIO_Port, VL1_XSHUT_Pin, GPIO_PIN_SET); //start up the sensor.
	//	HAL_Delay(100);  //24

	if(!VL53L0X_InitSensor(lidar, newAddr)) //attempt to initialise it with the necessary settings for normal operation. Returns 0 if fail, 1 if success. // hmmm, could use a <<1
	{
		status3 = HAL_I2C_IsDeviceReady(&hi2c1, newAddr , 1, 1000 );
		printf("Failed to initialize\r\n");
	}
	else
	{
		printf("Successfully initialized\r\n");
	}
}


void _Error_Handler(char *file, int line)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	char msg[256];
	sprintf(msg,"%s,%d", file,line);
	/* User can add his own implementation to report the HAL error return state */
	//	while(1)
	//	{
	//	}
	return;
	/* USER CODE END Error_Handler_Debug */
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


static void SensorConfig_All(void)
{
	volatile HAL_StatusTypeDef status1;
	uint8_t I2C_Dev1_addr = 0b11101110;
	MCP23017_HandleTypeDef hmcp;
	uint8_t numTrays = 1;
	uint8_t traySels[numTrays];
	traySels[0] = Tray0_Sels;
	traySels[1] = Tray1_Sels;
	traySels[2] = Tray2_Sels;
	traySels[3] = Tray3_Sels;
	traySels[4] = Tray4_Sels;
	traySels[5] = Tray5_Sels;
	traySels[6] = Tray6_Sels;
	traySels[7] = Tray7_Sels;



	uint8_t sensor1_newAddr = 0x36;
	uint8_t sensor2_newAddr = 0x30;
	uint8_t sensor3_newAddr = 0x33;
	uint8_t sensor4_newAddr = 0x39;
	uint8_t s5_newAddr = 0x42;
	uint8_t s6_newAddr = 0x45;
	uint8_t s7_newAddr = 0x48;
	uint8_t s8_newAddr = 0x51;
	uint8_t s9_newAddr = 0x54;
	uint8_t s10_newAddr = 0x57;
	uint8_t s11_newAddr = 0x60;
	uint8_t s12_newAddr = 0x63;
	uint8_t s13_newAddr = 0x66;
	uint8_t s14_newAddr = 0x69;
	uint8_t s15_newAddr = 0x71;
	uint8_t s16_newAddr = 0x75;

	uint8_t d3 = 0b00000001; // d3 is the selector for the MainBoardSide I2c Switch
	uint8_t d2 = 0;
	status1 = HAL_I2C_Master_Transmit(&hi2c1, I2C_Dev1_addr ,&d3, 1, 1000);
	status1 = HAL_I2C_Master_Receive(&hi2c1, I2C_Dev1_addr ,&d2, 1, 1000);

	mcp23017_init(&hmcp, &hi2c1, MCP23017_ADDRESS_27);
	mcp23017_iodir(&hmcp, MCP23017_PORTA, MCP23017_IODIR_ALL_OUTPUT);
	mcp23017_iodir(&hmcp, MCP23017_PORTB, MCP23017_IODIR_ALL_OUTPUT);


	hmcp.gpio[MCP23017_PORTB] = 0b11111111;//hmcp.gpio[MCP23017_PORTA];
	mcp23017_write_gpio(&hmcp, MCP23017_PORTB);
	//	}

	hmcp.gpio[MCP23017_PORTA] = 0b00000000;//hmcp.gpio[MCP23017_PORTA];
	mcp23017_write_gpio(&hmcp, MCP23017_PORTA);

	// RESET
	hmcp.gpio[MCP23017_PORTB] = 0b00000000;//hmcp.gpio[MCP23017_PORTA];
	mcp23017_write_gpio(&hmcp, MCP23017_PORTB);
	HAL_Delay(5000);
	// end Reset

	for(uint16_t ij =0; ij<numTrays; ij++){
		switch(ij)
		{
		case 0:
			d3 = 0b00000001; // d3 is the selector for the MainBoardSide I2c Switch
			break;
		case 1:
			d3 = 0b00000010; // d3 is the selector for the MainBoardSide I2c Switch
			break;
		case 2:
			d3 = 0b00000100; // d3 is the selector for the MainBoardSide I2c Switch
			break;
		case 3:
			d3 = 0b00001000; // d3 is the selector for the MainBoardSide I2c Switch
			break;
		case 4:
			d3 = 0b00010000; // d3 is the selector for the MainBoardSide I2c Switch
			break;
		case 5:
			d3 = 0b00100000; // d3 is the selector for the MainBoardSide I2c Switch
			break;
		case 6:
			d3 = 0b01000000; // d3 is the selector for the MainBoardSide I2c Switch
			break;
		case 7:
			d3 = 0b10000000; // d3 is the selector for the MainBoardSide I2c Switch
			break;
		}

		status1 = HAL_I2C_Master_Transmit(&hi2c1, I2C_Dev1_addr ,&d3, 1, 1000);
		status1 = HAL_I2C_Master_Receive(&hi2c1, I2C_Dev1_addr ,&d2, 1, 1000);

		mcp23017_init(&hmcp, &hi2c1, MCP23017_ADDRESS_27);
		mcp23017_iodir(&hmcp, MCP23017_PORTA, MCP23017_IODIR_ALL_OUTPUT);
		mcp23017_iodir(&hmcp, MCP23017_PORTB, MCP23017_IODIR_ALL_OUTPUT);


		hmcp.gpio[MCP23017_PORTB] = 0b11111111;//hmcp.gpio[MCP23017_PORTA];
		mcp23017_write_gpio(&hmcp, MCP23017_PORTB);
		//	}
		HAL_Delay(150);

		hmcp.gpio[MCP23017_PORTA] = 0b00000000;//hmcp.gpio[MCP23017_PORTA];
		mcp23017_write_gpio(&hmcp, MCP23017_PORTA);

		// RESET
		hmcp.gpio[MCP23017_PORTB] = 0b00000000;//hmcp.gpio[MCP23017_PORTA];
		mcp23017_write_gpio(&hmcp, MCP23017_PORTB);
		HAL_Delay(1000);

		for(uint16_t kk =0; kk<traySels[ij]; kk++){
			switch(kk){
			case 0:
				hmcp.gpio[MCP23017_PORTB] = 0b00000001;//hmcp.gpio[MCP23017_PORTA]   		// This releases the release line on the remote reset line of the proxim sensor.
				mcp23017_write_gpio(&hmcp, MCP23017_PORTB);
				HAL_Delay(1000);
				VL53L0X_Start(&sensor1, sensor1_newAddr);

				status1 = HAL_I2C_IsDeviceReady(&hi2c1, sensor1_newAddr , 1, 1000 );
				break;
			case 1:
				hmcp.gpio[MCP23017_PORTB] = 0b00000011;//hmcp.gpio[MCP23017_PORTA];
				mcp23017_write_gpio(&hmcp, MCP23017_PORTB);
				HAL_Delay(1000);
				VL53L0X_Start(&sensor2, sensor2_newAddr);
				break;
			case 2:
				hmcp.gpio[MCP23017_PORTB] = 0b00000111;//hmcp.gpio[MCP23017_PORTA];
				mcp23017_write_gpio(&hmcp, MCP23017_PORTB);
				HAL_Delay(1000);
				VL53L0X_Start(&sensor3, sensor3_newAddr);
				break;
			case 3:
				hmcp.gpio[MCP23017_PORTB] = 0b00001111;//hmcp.gpio[MCP23017_PORTA];
				mcp23017_write_gpio(&hmcp, MCP23017_PORTB);
				HAL_Delay(1000);
				VL53L0X_Start(&sensor4, sensor4_newAddr);
				break;
			case 4:
				hmcp.gpio[MCP23017_PORTB] = 0b00011111;//hmcp.gpio[MCP23017_PORTA];
				mcp23017_write_gpio(&hmcp, MCP23017_PORTB);
				HAL_Delay(1000);
				VL53L0X_Start(&s5, s5_newAddr);
				break;
			case 5:
				hmcp.gpio[MCP23017_PORTB] = 0b00111111;//hmcp.gpio[MCP23017_PORTA];
				mcp23017_write_gpio(&hmcp, MCP23017_PORTB);
				HAL_Delay(1000);
				VL53L0X_Start(&s6, s6_newAddr);
				break;

			case 6:
				hmcp.gpio[MCP23017_PORTB] = 0b01111111;//hmcp.gpio[MCP23017_PORTA];
				mcp23017_write_gpio(&hmcp, MCP23017_PORTB);
				HAL_Delay(1000);
				VL53L0X_Start(&s7, s7_newAddr);
				break;
			case 7:
				hmcp.gpio[MCP23017_PORTB] = 0b11111111;//hmcp.gpio[MCP23017_PORTA];
				mcp23017_write_gpio(&hmcp, MCP23017_PORTB);
				HAL_Delay(1000);
				VL53L0X_Start(&s8, s8_newAddr);
				break;

			}
		}
	}





}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
