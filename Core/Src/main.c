/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_filex.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "linked_list.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct{

    // calculated values
    float dec_longitude;
    float dec_latitude;
    float altitude_ft;

    // GGA - Global Positioning System Fixed Data
    float nmea_longitude;
    float nmea_latitude;
    float utc_time;
    char ns, ew;
    int lock;
    int satelites;
    float hdop;
    float msl_altitude;
    char msl_units;

    // RMC - Recommended Minimmum Specific GNS Data
    char rmc_status;
    float speed_k;
    float course_d;
    int date;

    // GLL
    char gll_status;

    // VTG - Course over ground, ground speed
    float course_t; // ground speed true
    char course_t_unit;
    float course_m; // magnetic
    char course_m_unit;
    char speed_k_unit;
    float speed_km; // speek km/hr
    char speed_km_unit;
} GPS_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//define const
#define I2C_Buffer_Size 						8
#define I2C_TimeOut 							200

//sensor buffer tamanho
#define SENSOR_Tam_Buffer 						64

//IMU define
#define MPU6050_port 							hi2c1
#define MPU6050_ADDR							0xD0 // 0x68 << 1
#define MPU6050_ACCEL_REG_ADDR	    			0x3B
#define MPU6050_TEMP_REG_ADDR	    			0x41
#define MPU6050_GYRO_REG_ADDR	    			0x43
#define MPU6050_WHO_AM_I_REG					0x75
#define MPU6050_PWR_MGMT_1						0x6B
#define MPU6050_SMPLRT_DIV						0x19
#define MPU6050_GYRO_CONFIG						0x1B
#define MPU6050_ACCEL_CONFIG					0x1C

//BPS
#define BME280_port 							hi2c1
#define BME280_ADDR								0xEC // 0x76 << 1

// Oversampling definitions
#define OSRS_OFF    							0x00
#define OSRS_1      							0x01
#define OSRS_2      							0x02
#define OSRS_4      							0x03
#define OSRS_8      							0x04
#define OSRS_16     							0x05

// MODE Definitions
#define MODE_SLEEP      						0x00
#define MODE_FORCED     						0x01
#define MODE_NORMAL     						0x03

// Standby Time
#define T_SB_0p5    							0x00
#define T_SB_62p5   							0x01
#define T_SB_125    							0x02
#define T_SB_250    							0x03
#define T_SB_500    							0x04
#define T_SB_1000   							0x05
#define T_SB_10     							0x06
#define T_SB_20     							0x07

// IIR Filter Coefficients
#define IIR_OFF     							0x00
#define IIR_2       							0x01
#define IIR_4       							0x02
#define IIR_8       							0x03
#define IIR_16      							0x04

// REGISTERS DEFINITIONS
#define ID_REG      							0xD0
#define RESET_REG  								0xE0
#define CTRL_HUM_REG    						0xF2
#define STATUS_REG      						0xF3
#define CTRL_MEAS_REG   						0xF4
#define CONFIG_REG      						0xF5
#define PRESS_MSB_REG   						0xF7

//usar2
#define USART2_BUFFER_SIZE          			768

//nmea
#define NMEA_BUFFER_SIZE						76
#define NMEA_BUFFER_CKS_SIZE					3

//INA
#define INA219_port 							hi2c1
#define INA219_ADDR_data 						0x41
#define INA219_ADDR								0x82 //INA219_ADDR_data << 1

#define	INA219_REG_CONFIG						0x00
#define	INA219_REG_SHUNTVOLTAGE					0x01
#define	INA219_REG_BUSVOLTAGE					0x02
#define	INA219_REG_POWER						0x03
#define	INA219_REG_CURRENT						0x04
#define	INA219_REG_CALIBRATION					0x05
//
#define INA219_CONFIG_RESET 					0x8000
//
#define INA219_CONFIG_BVOLTAGERANGE_16V			0x0000 // 0-16V Range
#define INA219_CONFIG_BVOLTAGERANGE_32V			0x2000 // 0-32V Range

#define	INA219_CONFIG_GAIN_1_40MV				0x0000  // Gain 1, 40mV Range
#define	INA219_CONFIG_GAIN_2_80MV				0x0800  // Gain 2, 80mV Range
#define	INA219_CONFIG_GAIN_4_160MV				0x1000 // Gain 4, 160mV Range
#define	INA219_CONFIG_GAIN_8_320MV				0x1800 // Gain 8, 320mV Range

#define	INA219_CONFIG_BADCRES_9BIT				0x0000  // 9-bit bus res = 0..511
#define	INA219_CONFIG_BADCRES_10BIT				0x0080 // 10-bit bus res = 0..1023
#define	INA219_CONFIG_BADCRES_11BIT				0x0100 // 11-bit bus res = 0..2047
#define	INA219_CONFIG_BADCRES_12BIT				0x0180 // 12-bit bus res = 0..4097
#define	INA219_CONFIG_BADCRES_12BIT_2S_1060US 	0x0480 // 2 x 12-bit bus samples averaged together
#define	INA219_CONFIG_BADCRES_12BIT_4S_2130US	0x0500 // 4 x 12-bit bus samples averaged together
#define	INA219_CONFIG_BADCRES_12BIT_8S_4260US	0x0580 // 8 x 12-bit bus samples averaged together
#define	INA219_CONFIG_BADCRES_12BIT_16S_8510US	0x0600 // 16 x 12-bit bus samples averaged together
#define	INA219_CONFIG_BADCRES_12BIT_32S_17MS	0x0680 // 32 x 12-bit bus samples averaged together
#define	INA219_CONFIG_BADCRES_12BIT_64S_34MS	0x0700 // 64 x 12-bit bus samples averaged together
#define	INA219_CONFIG_BADCRES_12BIT_128S_69MS	0x0780 // 128 x 12-bit bus samples averaged together

#define	INA219_CONFIG_SADCRES_9BIT_1S_84US		0x0000 // 1 x 9-bit shunt sample
#define	INA219_CONFIG_SADCRES_10BIT_1S_148US	0x0008 // 1 x 10-bit shunt sample
#define	INA219_CONFIG_SADCRES_11BIT_1S_276US	0x0010 // 1 x 11-bit shunt sample
#define	INA219_CONFIG_SADCRES_12BIT_1S_532US	0x0018 // 1 x 12-bit shunt sample
#define	INA219_CONFIG_SADCRES_12BIT_2S_1060US	0x0048 // 2 x 12-bit shunt samples averaged together
#define	INA219_CONFIG_SADCRES_12BIT_4S_2130US	0x0050 // 4 x 12-bit shunt samples averaged together
#define	INA219_CONFIG_SADCRES_12BIT_8S_4260US	0x0058 // 8 x 12-bit shunt samples averaged together
#define	INA219_CONFIG_SADCRES_12BIT_16S_8510US	0x0060 // 16 x 12-bit shunt samples averaged together
#define	INA219_CONFIG_SADCRES_12BIT_32S_17MS	0x0068 // 32 x 12-bit shunt samples averaged together
#define	INA219_CONFIG_SADCRES_12BIT_64S_34MS	0x0070 // 64 x 12-bit shunt samples averaged together
#define	INA219_CONFIG_SADCRES_12BIT_128S_69MS	0x0078 // 128 x 12-bit shunt samples averaged together

#define INA219_CONFIG_MODE_MASK					0x07
#define	INA219_CONFIG_MODE_POWERDOWN			0x00 /**< power down */
#define	INA219_CONFIG_MODE_SVOLT_TRIGGERED		0x01 /**< shunt voltage triggered */
#define	INA219_CONFIG_MODE_BVOLT_TRIGGERED		0x02 /**< bus voltage triggered */
#define	INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED	0x03 /**< shunt and bus voltage triggered */
#define	INA219_CONFIG_MODE_ADCOFF				0x04 /**< ADC off */
#define	INA219_CONFIG_MODE_SVOLT_CONTINUOUS		0x05 /**< shunt voltage continuous */
#define	INA219_CONFIG_MODE_BVOLT_CONTINUOUS		0x06 /**< bus voltage continuous */
#define	INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS 0x07



//define if
//debug
#define GPS_definido							1
#define IMU_definido							1
#define BPS_definido							1
#define INA_definido							1
#define TIM6_definido							1
#define TIM7_definido							0
#define TIM16_definido							0
#define TIM17_definido							1
#define UART_IDLE_definido						1
#define SD_definido								1
#define IWDG_definido							1
/*
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_16;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 2000;
  hiwdg.Init.EWI = 0;
*/


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;
I2C_HandleTypeDef hi2c4;

IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_NodeTypeDef Node_GPDMA1_Channel10;
DMA_QListTypeDef List_GPDMA1_Channel10;
DMA_HandleTypeDef handle_GPDMA1_Channel10;

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */
//rtc
RTC_DateTypeDef RTC_data;
RTC_TimeTypeDef RTC_tempo;

//gps
#if (GPS_definido == 1)
GPS_t GPS;

volatile uint8_t GPS_PARSE_GNRMC[NMEA_BUFFER_SIZE] = {0};
volatile char GPS_CHECKSUM_GNRMC[NMEA_BUFFER_CKS_SIZE] = {0};

volatile uint8_t GPS_PARSE_GNVTG[NMEA_BUFFER_SIZE] = {0};
volatile char GPS_CHECKSUM_GNVTG[NMEA_BUFFER_CKS_SIZE] = {0};

volatile uint8_t GPS_PARSE_GNGGA[NMEA_BUFFER_SIZE] = {0};
volatile char GPS_CHECKSUM_GNGGA[NMEA_BUFFER_CKS_SIZE] = {0};

volatile uint8_t GPS_PARSE_GNGSA[NMEA_BUFFER_SIZE] = {0};
volatile char GPS_CHECKSUM_GNGSA[NMEA_BUFFER_CKS_SIZE] = {0};

volatile uint8_t GPS_PARSE_GPGSV[NMEA_BUFFER_SIZE] = {0};
volatile char GPS_CHECKSUM_GPGSV[NMEA_BUFFER_CKS_SIZE] = {0};

volatile uint8_t GPS_PARSE_GNGLL[NMEA_BUFFER_SIZE] = {0};
volatile char GPS_CHECKSUM_GNGLL[NMEA_BUFFER_CKS_SIZE] = {0};

int8_t FLAG_GPS_VALIDANDO = 0;
#endif

//buffers
//uart2
#if (UART_IDLE_definido == 1)
volatile uint8_t RXBufferUSART2[USART2_BUFFER_SIZE] = {0};
volatile uint8_t MainBufferUSART2_0[USART2_BUFFER_SIZE] = {0};
volatile uint8_t MainBufferUSART2_1[USART2_BUFFER_SIZE] = {0};
volatile uint8_t MainBufferUSART2_2[USART2_BUFFER_SIZE] = {0};
volatile uint8_t ErrorBufferUSART2_0[USART2_BUFFER_SIZE] = {0};
volatile uint8_t ErrorBufferUSART2_1[USART2_BUFFER_SIZE] = {0};
volatile uint8_t ErrorBufferUSART2_2[USART2_BUFFER_SIZE] = {0};
volatile uint8_t RXBuffer_Index = 0;

int8_t FLAG_UART_LIVRE = 0;
int8_t FLAG_UART_BOOT = 1;
#endif

//i2c
volatile uint8_t I2C1_RegsBuffer[I2C_Buffer_Size] = {0};

//IMU variables
#if (IMU_definido == 1)
volatile int32_t IMU_Acc_X_RAW = 0;
volatile int32_t IMU_Acc_Y_RAW = 0;
volatile int32_t IMU_Acc_Z_RAW = 0;
volatile int32_t IMU_Temp_RAW = 0;
volatile int32_t IMU_Gyro_X_RAW = 0;
volatile int32_t IMU_Gyro_Y_RAW = 0;
volatile int32_t IMU_Gyro_Z_RAW = 0;
volatile float IMU_Ax = 0;
volatile float IMU_Ay = 0;
volatile float IMU_Az = 0;
volatile float IMU_T = 0;
volatile float IMU_Gx = 0;
volatile float IMU_Gy = 0;
volatile float IMU_Gz = 0;

volatile int16_t IMU_Acc_X_RAW_VET[SENSOR_Tam_Buffer] = {0};
volatile int16_t IMU_Acc_Y_RAW_VET[SENSOR_Tam_Buffer] = {0};
volatile int16_t IMU_Acc_Z_RAW_VET[SENSOR_Tam_Buffer] = {0};
volatile int16_t IMU_Temp_RAW_VET[SENSOR_Tam_Buffer] = {0};
volatile int16_t IMU_Gyro_X_RAW_VET[SENSOR_Tam_Buffer] = {0};
volatile int16_t IMU_Gyro_Y_RAW_VET[SENSOR_Tam_Buffer] = {0};
volatile int16_t IMU_Gyro_Z_RAW_VET[SENSOR_Tam_Buffer] = {0};
#endif

volatile uint8_t Contador_Sensor_Index = 0;

//bps
#if (BPS_definido == 1)
const double BPS_Lb = -0.0065;
const double BPS_PSL = 101325;
const long double BPS_Const = 0.1902632365;
volatile int64_t BPS_Temperatura_RAW = 0;
volatile int64_t BPS_Pressao_RAW = 0;
volatile int64_t BPS_Umidade_RAW = 0;
volatile uint16_t BPS_Dig_T1,  \
    	 BPS_Dig_P1, \
         BPS_Dig_H1, BPS_Dig_H3;
volatile int16_t  BPS_Dig_T2, BPS_Dig_T3, \
         BPS_Dig_P2, BPS_Dig_P3, BPS_Dig_P4, BPS_Dig_P5, BPS_Dig_P6, BPS_Dig_P7, BPS_Dig_P8, BPS_Dig_P9, \
		 BPS_Dig_H2,  BPS_Dig_H4, BPS_Dig_H5, BPS_Dig_H6;
volatile int32_t BPS_BPS_Temperature_Fina = 0;
volatile float BPS_Temperatura = 0;
volatile float BPS_Pressao = 0;
volatile float BPS_Umidade = 0;
volatile int32_t BPS_Temperatura_RAW_VET[SENSOR_Tam_Buffer] = {0};
volatile int32_t BPS_Pressao_RAW_VET[SENSOR_Tam_Buffer] = {0};
volatile int32_t BPS_Umidade_RAW_VET[SENSOR_Tam_Buffer] = {0};
volatile double BPS_Altitude = 0;
#endif

//INA
#if (INA_definido == 1)
const uint16_t ina219_calibrationValue = 8192;
const int16_t ina219_currentDivider_mA = 20;
const int16_t ina219_powerMultiplier_mW = 1.0f;

volatile uint16_t INA219_vBus_RAW_VET[SENSOR_Tam_Buffer] = {0};
volatile uint16_t INA219_vShunt_RAW_VET[SENSOR_Tam_Buffer] = {0};
volatile uint16_t INA219_Corrente_RAW_VET[SENSOR_Tam_Buffer] = {0};
volatile uint32_t INA219_vBus_RAW = 0;
volatile uint32_t INA219_vShunt_RAW = 0;
volatile uint32_t INA219_Corrente_RAW = 0;
volatile double INA219_vBus = 0;
volatile double INA219_vShunt = 0;
volatile double INA219_Corrente = 0;
#endif


//RTC
volatile uint8_t RTC_Segundos = 0;
volatile uint8_t RTC_Minutos = 0;
volatile uint8_t RTC_Horas = 0;

//sd
#if (SD_definido == 1)
FX_MEDIA        sdio_disk;
FX_FILE         fx_file_dados;
const char nome_arquivo1[] = "Dados.txt";
volatile uint32_t media_memory[512/sizeof(uint32_t)];
const char SD_BOOT_CABECA[] = "\n----BOOT LOG----";
const char SD_ROTINA_CABECA[] = "\n\n\n\n---------- Rotina ----------";
const char SD_ROTINA_GPS_CABECA[] = "\n\n----- Dados GPS -----";
const char SD_ROTINA_GPS_ERROR_CABECA[] = "\n\n----- Dados GPS ERROR -----";
const char SD_ROTINA_IMU_CABECA[] = "\n\n----- Dados IMU -----";
const char SD_ROTINA_BPS_CABECA[] = "\n\n----- Dados BPS -----";
const char SD_ROTINA_INA_CABECA[] = "\n\n----- Dados INA -----";
#endif

volatile uint8_t FLAG_BOOT_SD = 1;
volatile uint8_t FLAG_GPS_SD_ERROR = 0;
volatile uint32_t Contador_GPS_ERRO = 0;
volatile uint32_t Contador_SD_ROTINA = 0;
volatile uint8_t FLAG_SD_PROCESSANDO = 0;
volatile uint8_t FLAG_LED_50ms = 0;
volatile uint8_t FLAG_LED_200ms = 0;
volatile uint32_t Contador_TIM6 = 0;
volatile uint32_t Contador_TIM7 = 0;
volatile uint32_t Contador_TIM16 = 0;
volatile uint32_t Contador_TIM17 = 0;
volatile uint16_t Contador_TIM6_1hz = 0;
volatile uint32_t Contador_SENSOR = 0;
volatile uint16_t Contador_TIM17_50ms = 0;
volatile uint16_t Contador_TIM17_200ms = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_GPDMA1_Init(void);
static void MX_MEMORYMAP_Init(void);
static void MX_ICACHE_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_I2C4_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_IWDG_Init(void);
static void MX_LPUART1_UART_Init(void);
/* USER CODE BEGIN PFP */
//imu
void clear_buffers(void);
void IMU_Init(void);
void IMU_Accel_Read_RAW(void);
void IMU_Gyro_Read_RAW(void);
void IMU_Temp_Read_RAW(void);

//bps
void BPS_Init(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
uint32_t BPS_Pressao_Comp(int32_t);
uint32_t BPS_Umidade_Comp(int32_t);
int32_t BPS_Temperatura_Comp(int32_t);
void BPS_WakeUP(void);
void BPS_Calibracao(void);
void BPS_Read_RAW(void);

//INA
void INA219_Reset(void);
void INA219_Calibracao(void);
void INA219_Read_RAW(void);

//gps
#if (UART_IDLE_definido == 1)
void IniciaEscuta_USART2(void);
#endif
void GPS_Index(void);
int GPS_VALIDANDO_PARSE_XNMEA(char *STR_BUFFER, uint16_t);
void GPS_parse(void);
#if (GPS_definido == 1)
void memset0_Buff_GPS(void);
#endif
void memset0_Buff_Sensores(void);
float GPS_nmea_to_dec(float, char);

//sd
void SD_LOG_BOOT(void);
void SD_LOG_rotina(void);

//handler erro
void SD_Error_Handler(void);
void UART2_Error_Handler(void);
void GPS_Error_Handler(void);
void IMU_Error_Handler(void);
void BPS_Error_Handler(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	/*
	sTime.Hours = 0x14;
	sTime.Minutes = 0x0;
	sTime.Seconds = 0x0;
	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK){
	  Error_Handler();
	}
	sDate.WeekDay = RTC_WEEKDAY_TUESDAY;
	sDate.Month = RTC_MONTH_AUGUST;
	sDate.Date = 0x1;
	sDate.Year = 0x17;

	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK){
	  Error_Handler();
	}
	*/

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
  MX_GPDMA1_Init();
  MX_MEMORYMAP_Init();
  MX_ICACHE_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_I2C4_Init();
  MX_SDMMC1_SD_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_FileX_Init();
  MX_RTC_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_IWDG_Init();
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LED_DEBUG_PORT, LED_DEBUG_PIN, GPIO_PIN_SET);

  GPS.ns = 'S';
  GPS.ew = 'W';

#if (IWDG_definido == 1)
  HAL_IWDG_Refresh(&hiwdg);
#endif

  HAL_RTC_GetTime(&hrtc, &RTC_tempo, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &RTC_data, RTC_FORMAT_BIN);

  RTC_Segundos = RTC_tempo.Seconds;
  RTC_Minutos = RTC_tempo.Minutes;
  RTC_Horas = RTC_tempo.Hours;

#if (SD_definido == 1)
	  SD_LOG_BOOT();
	  HAL_Delay(10);
#endif

#if (IWDG_definido == 1)
  HAL_IWDG_Refresh(&hiwdg);
#endif

#if (IMU_definido == 1)
	  IMU_Init();
	  HAL_Delay(10);
#endif

#if (IWDG_definido == 1)
  HAL_IWDG_Refresh(&hiwdg);
#endif

#if (BPS_definido == 1)
	  BPS_Init(OSRS_2, OSRS_16, OSRS_1, MODE_NORMAL, T_SB_0p5, IIR_16);
	  HAL_Delay(10);
#endif

#if (IWDG_definido == 1)
  HAL_IWDG_Refresh(&hiwdg);
#endif

#if (INA_definido == 1)
	  INA219_Reset();
	  INA219_Calibracao();
	  HAL_Delay(10);
#endif

#if (UART_IDLE_definido == 1)
	  MX_YourQueueName_Config();
	  __HAL_LINKDMA(&huart2, hdmarx, handle_GPDMA1_Channel10);

	  if (HAL_DMAEx_List_LinkQ(&handle_GPDMA1_Channel10, &List_GPDMA1_Channel10) != HAL_OK) {
		  Error_Handler();
	  }
#endif

#if (IWDG_definido == 1)
	HAL_IWDG_Refresh(&hiwdg);
#endif

#if (TIM6_definido == 1)
	HAL_TIM_Base_Start_IT(&htim6);
#endif

#if (TIM7_definido == 1)
	HAL_TIM_Base_Start_IT(&htim7);
#endif

#if (TIM16_definido == 1)
	HAL_TIM_Base_Start_IT(&htim16);
#endif

#if (TIM17_definido == 1)
	HAL_TIM_Base_Start_IT(&htim17);
#endif


HAL_GPIO_WritePin(LED_DEBUG_PORT, LED_DEBUG_PIN, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
#if (IWDG_definido == 1)
  HAL_IWDG_Refresh(&hiwdg);
#endif
	HAL_RTC_GetTime(&hrtc, &RTC_tempo, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &RTC_data, RTC_FORMAT_BIN);

	RTC_Segundos = RTC_tempo.Seconds;
	RTC_Minutos = RTC_tempo.Minutes;
	RTC_Horas = RTC_tempo.Hours;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.LSIDiv = RCC_LSI_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV4;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();

  /** Enables the Clock Security System
  */
  HAL_RCCEx_EnableLSECSS();
}

/**
  * @brief GPDMA1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPDMA1_Init(void)
{

  /* USER CODE BEGIN GPDMA1_Init 0 */

  /* USER CODE END GPDMA1_Init 0 */

  /* Peripheral clock enable */
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  /* GPDMA1 interrupt Init */
    HAL_NVIC_SetPriority(GPDMA1_Channel10_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel10_IRQn);

  /* USER CODE BEGIN GPDMA1_Init 1 */

  /* USER CODE END GPDMA1_Init 1 */
  /* USER CODE BEGIN GPDMA1_Init 2 */

  /* USER CODE END GPDMA1_Init 2 */

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
  hi2c1.Init.Timing = 0x00F07BFF;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00F07BFF;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00F07BFF;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x00F07BFF;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */
#if (IWDG_definido == 1)

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_16;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 2000;
  hiwdg.Init.EWI = 0;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */
#endif

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 209700;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
  huart2.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief MEMORYMAP Initialization Function
  * @param None
  * @retval None
  */
static void MX_MEMORYMAP_Init(void)
{

  /* USER CODE BEGIN MEMORYMAP_Init 0 */

  /* USER CODE END MEMORYMAP_Init 0 */

  /* USER CODE BEGIN MEMORYMAP_Init 1 */

  /* USER CODE END MEMORYMAP_Init 1 */
  /* USER CODE BEGIN MEMORYMAP_Init 2 */

  /* USER CODE END MEMORYMAP_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_PrivilegeStateTypeDef privilegeState = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  hrtc.Init.BinMode = RTC_BINARY_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  privilegeState.rtcPrivilegeFull = RTC_PRIVILEGE_FULL_NO;
  privilegeState.backupRegisterPrivZone = RTC_PRIVILEGE_BKUP_ZONE_NONE;
  privilegeState.backupRegisterStartZone2 = RTC_BKP_DR0;
  privilegeState.backupRegisterStartZone3 = RTC_BKP_DR0;
  if (HAL_RTCEx_PrivilegeModeSet(&hrtc, &privilegeState) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;
  if (HAL_SD_Init(&hsd1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  SPI_AutonomousModeConfTypeDef HAL_SPI_AutonomousMode_Cfg_Struct = {0};

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x7;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi1.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi1.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerState = SPI_AUTO_MODE_DISABLE;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerSelection = SPI_GRP1_GPDMA_CH0_TCF_TRG;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerPolarity = SPI_TRIG_POLARITY_RISING;
  if (HAL_SPIEx_SetConfigAutonomousMode(&hspi1, &HAL_SPI_AutonomousMode_Cfg_Struct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  SPI_AutonomousModeConfTypeDef HAL_SPI_AutonomousMode_Cfg_Struct = {0};

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x7;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi2.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi2.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerState = SPI_AUTO_MODE_DISABLE;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerSelection = SPI_GRP1_GPDMA_CH0_TCF_TRG;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerPolarity = SPI_TRIG_POLARITY_RISING;
  if (HAL_SPIEx_SetConfigAutonomousMode(&hspi2, &HAL_SPI_AutonomousMode_Cfg_Struct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 2501;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 3201;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 5000;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 6397;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 5000;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 6397;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 801;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 2001;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12
                          |GPIO_PIN_15|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC6 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA8 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB12
                           PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

#if (SD_definido == 1)
void SD_LOG_BOOT(void){
#if (IWDG_definido == 1)
  HAL_IWDG_Refresh(&hiwdg);
#endif

	HAL_GPIO_WritePin(LED_DEBUG_PORT, LED_DEBUG_PIN, GPIO_PIN_SET);
	UINT status;
	volatile char SD_BOOT_TEMPO[22] = {0};
	sprintf((char*)SD_BOOT_TEMPO, "\nTempo: %02d:%02d:%02d", (uint8_t)RTC_Horas, (uint8_t)RTC_Minutos, (uint8_t)RTC_Segundos);
	fflush(stdout);


	status =  fx_media_open(&sdio_disk, "LTSAT", fx_stm32_sd_driver, 0, (VOID *) media_memory, sizeof(media_memory));
	/* verifica status */
	if (status != FX_SUCCESS){
		SD_Error_Handler();
	}

	/* Cria arquivo de boot */
	status =  fx_file_create(&sdio_disk, (char*)nome_arquivo1);
	/* Verifica status da criação  */
	if (status != FX_SUCCESS){
		/* verifica se já tem flag de criação sendo usada  */
		if (status != FX_ALREADY_CREATED){
			SD_Error_Handler();
		}
	}

	/* Abre arquivo  */
	status =  fx_file_open(&sdio_disk, &fx_file_dados, (char*)nome_arquivo1, FX_OPEN_FOR_WRITE);
	/* verifica status  */
	if (status != FX_SUCCESS){
		SD_Error_Handler();
	}

	/* vai para o topo do arquivo  */
	//status =  fx_file_seek(&fx_file_dados, 0);
	/* verifica status  */
	//if (status != FX_SUCCESS){
		//Error_Handler();
	//}

	/* escvreve no arquivo  */
	status =  fx_file_write(&fx_file_dados, (char*)SD_BOOT_CABECA, sizeof(SD_BOOT_CABECA));
	/* verifica status  */
	if (status != FX_SUCCESS){
		SD_Error_Handler();
	}

	/* fechar arquivo */
	status =  fx_file_close(&fx_file_dados);
	/* verifica status  */
	if (status != FX_SUCCESS){
		SD_Error_Handler();
	}

	/* flush da memoria */
	status = fx_media_flush(&sdio_disk);
	/* verifica status flush  */
	if (status != FX_SUCCESS){
		SD_Error_Handler();
	}

	/* Abre arquivo  */
	status =  fx_file_open(&sdio_disk, &fx_file_dados, (char*)nome_arquivo1, FX_OPEN_FOR_WRITE);
	/* verifica status  */
	if (status != FX_SUCCESS){
		SD_Error_Handler();
	}

	/* escvreve no arquivo  */
	status =  fx_file_write(&fx_file_dados, (char*)SD_BOOT_TEMPO, sizeof(SD_BOOT_TEMPO));
	/* verifica status  */
	if (status != FX_SUCCESS){
		SD_Error_Handler();
	}

	/* fecha arquivo  */
	status =  fx_file_close(&fx_file_dados);
	/* verifica status  */
	if (status != FX_SUCCESS){
		SD_Error_Handler();
	}

	/* flush da memoria */
	status = fx_media_flush(&sdio_disk);
	/* verifica status flush  */
	if (status != FX_SUCCESS){
		SD_Error_Handler();
	}

	FLAG_BOOT_SD = 0;
	HAL_GPIO_WritePin(LED_DEBUG_PORT, LED_DEBUG_PIN, GPIO_PIN_RESET);
}
#endif

#if (SD_definido == 1)
void SD_LOG_rotina(){
	FLAG_SD_PROCESSANDO = 1;

#if (IWDG_definido == 1)
  HAL_IWDG_Refresh(&hiwdg);
#endif

	//HAL_GPIO_WritePin(LED_DEBUG_PORT, LED_DEBUG_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(OSC_DEBUG_PORT, OSC_DEBUG_PIN, GPIO_PIN_SET);
	UINT status;
	volatile char SD_ROTINA_TEMPO[22] = {0};
	volatile char SD_ROTINA_DEBUG[96] = {0};
	volatile char SD_ROTINA_DEBUG1[160] = {0};
	sprintf((char*)SD_ROTINA_TEMPO, "\nTempo: %02d:%02d:%02d", (uint8_t)RTC_Horas, (uint8_t)RTC_Minutos, (uint8_t)RTC_Segundos);
	fflush(stdout);
	sprintf((char*)SD_ROTINA_DEBUG, "\nDebug Contadores - SD:%lu GPS_E:%lu Sensor:%lu TIM6:%lu TIM7:%lu TIM16:%lu TIM17:%lu", (uint32_t)Contador_SD_ROTINA, (uint32_t)Contador_GPS_ERRO, (uint32_t)Contador_SENSOR, (uint32_t)Contador_TIM6, (uint32_t)Contador_TIM7, (uint32_t)Contador_TIM16, (uint32_t)Contador_TIM17);
	fflush(stdout);
	//sprintf((char*)SD_ROTINA_DEBUG1, "\nDebug Flag - | F_LED_200ms:%d | F_LED_50ms:%d | F_UART_LIVRE:%d | F__SD_PROCESSANDO:%d | F_GPS_VALIDANDO:%d | F_GPS_SD_ERROR:%d | RXBuffer_Index:%d", (uint8_t)FLAG_LED_200ms, (uint8_t)FLAG_LED_50ms, (uint8_t)FLAG_UART_LIVRE, (uint8_t)FLAG_SD_PROCESSANDO, (uint8_t)FLAG_GPS_VALIDANDO, (uint8_t)FLAG_GPS_SD_ERROR, (uint8_t)RXBuffer_Index);
	//fflush(stdout);


	/* Abre arquivo  */
	status =  fx_file_open(&sdio_disk, &fx_file_dados, (char*)nome_arquivo1, FX_OPEN_FOR_WRITE);
	/* verifica status  */
	if (status != FX_SUCCESS){
		SD_Error_Handler();
	}

	/* escvreve no arquivo  */
	status =  fx_file_write(&fx_file_dados, (char*)SD_ROTINA_CABECA, sizeof(SD_ROTINA_CABECA));
	/* verifica status  */
	if (status != FX_SUCCESS){
		SD_Error_Handler();
	}

	/* escvreve no arquivo  */
	status =  fx_file_write(&fx_file_dados, (char*)SD_ROTINA_TEMPO, sizeof(SD_ROTINA_TEMPO));
	/* verifica status  */
	if (status != FX_SUCCESS){
		SD_Error_Handler();
	}

	/* escvreve no arquivo  */
	status =  fx_file_write(&fx_file_dados, (char*)SD_ROTINA_DEBUG, sizeof(SD_ROTINA_DEBUG));
	/* verifica status  */
	if (status != FX_SUCCESS){
		SD_Error_Handler();
	}

	/* escvreve no arquivo  */
	status =  fx_file_write(&fx_file_dados, (char*)SD_ROTINA_DEBUG1, sizeof(SD_ROTINA_DEBUG1));
	/* verifica status  */
	if (status != FX_SUCCESS){
		SD_Error_Handler();
	}

#if (IMU_definido == 1)
	volatile char resultado_IMU[128] = {0};
	sprintf((char*)resultado_IMU, "\nAx:%.3f | Ay:%.3f | Az:%.3f | T:%.1f | Gx:%.3f | Gy:%.3f | Gz:%.3f", (float)IMU_Ax, (float)IMU_Ay, (float)IMU_Az, (float)IMU_T, (float)IMU_Gx, (float)IMU_Gy, (float)IMU_Gz);
	fflush(stdout);

	/* escvreve no arquivo  */
	status =  fx_file_write(&fx_file_dados, (char*)SD_ROTINA_IMU_CABECA, sizeof(SD_ROTINA_IMU_CABECA));
	/* verifica status  */
	if (status != FX_SUCCESS){
		SD_Error_Handler();
	}

	/* escvreve no arquivo  */
	status =  fx_file_write(&fx_file_dados, (char*)resultado_IMU, sizeof(resultado_IMU));
	/* verifica status  */
	if (status != FX_SUCCESS){
		SD_Error_Handler();
	}
#endif

#if (BPS_definido == 1)
	volatile char resultado_BPS[64] = {0};
	sprintf((char*)resultado_BPS, "\nP:%f | A:%.3lf | U:%.1f | T:%.1f", (float)BPS_Pressao, (double)BPS_Altitude, (float)BPS_Umidade, (float)BPS_Temperatura);
	fflush(stdout);

	/* escvreve no arquivo  */
	status =  fx_file_write(&fx_file_dados, (char*)SD_ROTINA_BPS_CABECA, sizeof(SD_ROTINA_BPS_CABECA));
	/* verifica status  */
	if (status != FX_SUCCESS){
		SD_Error_Handler();
	}

	/* escvreve no arquivo  */
	status =  fx_file_write(&fx_file_dados, (char*)resultado_BPS, sizeof(resultado_BPS));
	/* verifica status  */
	if (status != FX_SUCCESS){
		SD_Error_Handler();
	}
#endif

#if (INA_definido == 1)
	volatile char resultado_INA[64] = {0};
	sprintf((char*)resultado_INA, "\nvB:%.3lf | vS:%.3lf | i:%.3lf", (double)INA219_vBus, (double)INA219_vShunt, (double)INA219_Corrente);
	fflush(stdout);

	/* escreve no arquivo  */
	status =  fx_file_write(&fx_file_dados, (char*)SD_ROTINA_INA_CABECA, sizeof(SD_ROTINA_INA_CABECA));
	/* verifica status  */
	if (status != FX_SUCCESS){
		SD_Error_Handler();
	}

	/* escvreve no arquivo  */
	status =  fx_file_write(&fx_file_dados, (char*)resultado_INA, sizeof(resultado_INA));
	/* verifica status  */
	if (status != FX_SUCCESS){
		SD_Error_Handler();
	}
#endif

	/* fechar arquivo */
	status =  fx_file_close(&fx_file_dados);
	/* verifica status  */
	if (status != FX_SUCCESS){
		SD_Error_Handler();
	}

	/* flush da memoria */
	status = fx_media_flush(&sdio_disk);
	/* verifica status flush  */
	if (status != FX_SUCCESS){
		SD_Error_Handler();
	}

#if (GPS_definido == 1)
	volatile char resultado_GPS[192] = {0};
	sprintf((char*)resultado_GPS, "\nGPS: Lat_nmea:%f NS:%c Long_nmea:%f EW:%c Lat_dec:%f Long_dec:%f Alt:%.3f UTC:%.2f Q_Sat:%d", GPS.nmea_latitude, GPS.ns, GPS.nmea_longitude, GPS.ew, GPS.dec_latitude, GPS.dec_longitude, GPS.msl_altitude, GPS.utc_time, GPS.satelites);
	fflush(stdout);

	/* Abre arquivo  */
	status =  fx_file_open(&sdio_disk, &fx_file_dados, (char*)nome_arquivo1, FX_OPEN_FOR_WRITE);
	/* verifica status  */
	if (status != FX_SUCCESS){
		SD_Error_Handler();
	}

	/* escvreve no arquivo  */
	status =  fx_file_write(&fx_file_dados, (char*)SD_ROTINA_GPS_CABECA, sizeof(SD_ROTINA_GPS_CABECA));
	/* verifica status  */
	if (status != FX_SUCCESS){
		SD_Error_Handler();
	}

	/* escvreve no arquivo  */
	status =  fx_file_write(&fx_file_dados, (char*)resultado_GPS, sizeof(resultado_GPS));
	/* verifica status  */
	if (status != FX_SUCCESS){
		SD_Error_Handler();
	}

	/* escvreve no arquivo  */
	status =  fx_file_write(&fx_file_dados, "\n", 2);
	/* verifica status  */
	if (status != FX_SUCCESS){
		SD_Error_Handler();
	}

	if (FLAG_GPS_SD_ERROR == 1){
		FLAG_GPS_SD_ERROR = 0;

		/* fechar arquivo */
		status =  fx_file_close(&fx_file_dados);
		/* verifica status  */
		if (status != FX_SUCCESS){
			SD_Error_Handler();
		}

		/* flush da memoria */
		status = fx_media_flush(&sdio_disk);
		/* verifica status flush  */
		if (status != FX_SUCCESS){
			SD_Error_Handler();
		}

		/* Abre arquivo  */
		status =  fx_file_open(&sdio_disk, &fx_file_dados, (char*)nome_arquivo1, FX_OPEN_FOR_WRITE);
		/* verifica status  */
		if (status != FX_SUCCESS){
			SD_Error_Handler();
		}

		/* escvreve no arquivo  */
		status =  fx_file_write(&fx_file_dados, (char*)SD_ROTINA_GPS_ERROR_CABECA, sizeof(SD_ROTINA_GPS_ERROR_CABECA));
		/* verifica status  */
		if (status != FX_SUCCESS){
			SD_Error_Handler();
		}

		/* escvreve no arquivo  */
		status =  fx_file_write(&fx_file_dados, "\n", 2);
		/* verifica status  */
		if (status != FX_SUCCESS){
			SD_Error_Handler();
		}

#if (UART_IDLE_definido == 1)
		if (RXBuffer_Index == 0){
			/* escvreve no arquivo  */
			status =  fx_file_write(&fx_file_dados, (char*)ErrorBufferUSART2_2, USART2_BUFFER_SIZE);
			/* verifica status  */
			if (status != FX_SUCCESS){
				SD_Error_Handler();
			}
		} else if (RXBuffer_Index == 1){
			/* escvreve no arquivo  */
			status =  fx_file_write(&fx_file_dados, (char*)ErrorBufferUSART2_0, USART2_BUFFER_SIZE);
			/* verifica status  */
			if (status != FX_SUCCESS){
				SD_Error_Handler();
			}
		} else if (RXBuffer_Index == 2){
			/* escvreve no arquivo  */
			status =  fx_file_write(&fx_file_dados, (char*)ErrorBufferUSART2_1, USART2_BUFFER_SIZE);
			/* verifica status  */
			if (status != FX_SUCCESS){
				SD_Error_Handler();
			}
		}
#endif

		/* escvreve no arquivo  */
		status =  fx_file_write(&fx_file_dados, "\n", 2);
		/* verifica status  */
		if (status != FX_SUCCESS){
			SD_Error_Handler();
		}

	}
#endif


	/* fechar arquivo */
	status =  fx_file_close(&fx_file_dados);
	/* verifica status  */
	if (status != FX_SUCCESS){
		SD_Error_Handler();
	}

	/* flush da memoria */
	status = fx_media_flush(&sdio_disk);
	/* verifica status flush  */
	if (status != FX_SUCCESS){
		SD_Error_Handler();
	}

	//HAL_GPIO_WritePin(LED_DEBUG_PORT, LED_DEBUG_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OSC_DEBUG_PORT, OSC_DEBUG_PIN, GPIO_PIN_RESET);
}
#endif

void IMU_Init(void) {
	uint8_t data;
	uint8_t check;

	HAL_I2C_Mem_Read(&MPU6050_port, MPU6050_ADDR, MPU6050_WHO_AM_I_REG, 1, &check, 1, I2C_TimeOut);

	if (check == 0x68) {
		/* clock 8 MHz */
		data = 0x00;
		if (HAL_I2C_Mem_Write(&MPU6050_port, MPU6050_ADDR, MPU6050_PWR_MGMT_1, 1, &data, 1, I2C_TimeOut) != HAL_OK) {
			IMU_Error_Handler();
		}

		/* rate to 8kHz*/
		data = 0x07;
		if (HAL_I2C_Mem_Write(&MPU6050_port, MPU6050_ADDR, MPU6050_SMPLRT_DIV, 1, &data, 1, I2C_TimeOut) != HAL_OK) {
			IMU_Error_Handler();
		}


		/* escala gyro +/-500º/s */
		data = 0x01;
		if (HAL_I2C_Mem_Write(&MPU6050_port, MPU6050_ADDR, MPU6050_GYRO_CONFIG, 1, &data, 1, I2C_TimeOut) != HAL_OK) {
			IMU_Error_Handler();
		}

		/* escala acc +/-16g */
		data = 0x03;
		if (HAL_I2C_Mem_Write(&MPU6050_port, MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 1, &data, 1, I2C_TimeOut) != HAL_OK) {
			IMU_Error_Handler();
		}
	} else {
		IMU_Error_Handler();
	}
}

void BPS_Init(uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h, uint8_t mode, uint8_t t_sb, uint8_t filter)
{
	uint8_t check;

	HAL_I2C_Mem_Read(&hi2c1, BME280_ADDR, ID_REG, 1, &check, 1, I2C_TimeOut);

	if (check == 0x60){
		BPS_Calibracao();

		uint8_t datatowrite = 0;
		uint8_t datacheck = 0;

		// Reinicia BPS
		datatowrite = 0xB6;
		if (HAL_I2C_Mem_Write(&BME280_port, BME280_ADDR, RESET_REG, 1, &datatowrite, 1, I2C_TimeOut) != HAL_OK) {
			//BPS_Error_Handler();
		}

		HAL_Delay(200);

		//oversampling da umidade
		datatowrite = osrs_h;
		if (HAL_I2C_Mem_Write(&BME280_port, BME280_ADDR, CTRL_HUM_REG, 1, &datatowrite, 1, I2C_TimeOut) != HAL_OK) {
			BPS_Error_Handler();
		}

		HAL_Delay(200);

		HAL_I2C_Mem_Read(&BME280_port, BME280_ADDR, CTRL_HUM_REG, 1, &datacheck, 1, I2C_TimeOut);
		if (datacheck != datatowrite) {
			BPS_Error_Handler();
		}

		// write the standby time and IIR filter coeff to 0xF5
		datatowrite = (t_sb <<5) |(filter << 2);
		if (HAL_I2C_Mem_Write(&BME280_port, BME280_ADDR, CONFIG_REG, 1, &datatowrite, 1, I2C_TimeOut) != HAL_OK) {
			BPS_Error_Handler();
		}

		HAL_Delay (200);

		HAL_I2C_Mem_Read(&BME280_port, BME280_ADDR, CONFIG_REG, 1, &datacheck, 1, I2C_TimeOut);
		if (datacheck != datatowrite) {
			BPS_Error_Handler();
		}


		// oversampling pressao e temperatura
		datatowrite = (osrs_t <<5) |(osrs_p << 2) | mode;
		if (HAL_I2C_Mem_Write(&BME280_port, BME280_ADDR, CTRL_MEAS_REG, 1, &datatowrite, 1, I2C_TimeOut) != HAL_OK) {
			BPS_Error_Handler();
		}

		HAL_Delay (200);

		HAL_I2C_Mem_Read(&BME280_port, BME280_ADDR, CTRL_MEAS_REG, 1, &datacheck, 1, I2C_TimeOut);
		if (datacheck != datatowrite) {
			BPS_Error_Handler();
		}
	} else {
		BPS_Error_Handler();
	}

}

#if (BPS_definido == 1)
uint32_t BPS_Pressao_Comp(int32_t adc_P)
{
	int64_t var1, var2, p;
	var1 = ((int64_t)BPS_BPS_Temperature_Fina) - 128000;
	var2 = var1 * var1 * (int64_t)BPS_Dig_P6;
	var2 = var2 + ((var1*(int64_t)BPS_Dig_P5)<<17);
	var2 = var2 + (((int64_t)BPS_Dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)BPS_Dig_P3)>>8) + ((var1 * (int64_t)BPS_Dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)BPS_Dig_P1)>>33;
	if (var1 == 0) {
		BPS_Error_Handler();
	}
	p = 1048576-adc_P;
	p = (((p<<31)-var2)*3125)/var1;
	var1 = (((int64_t)BPS_Dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)BPS_Dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)BPS_Dig_P7)<<4);
	return (uint32_t)p;
}

uint32_t BPS_Umidade_Comp(int32_t adc_H)
{
	int32_t v_x1_u32r;
	v_x1_u32r = (BPS_BPS_Temperature_Fina - ((int32_t)76800));
	v_x1_u32r = (((((adc_H << 14) - (((int32_t)BPS_Dig_H4) << 20) - (((int32_t)BPS_Dig_H5) *\
			v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r *\
					((int32_t)BPS_Dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)BPS_Dig_H3)) >> 11) +\
							((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)BPS_Dig_H2) +\
					8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *\
			((int32_t)BPS_Dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	return (uint32_t)(v_x1_u32r>>12);
}

int32_t BPS_Temperatura_Comp(int32_t adc_T)
{
	int32_t var1, var2, T;
	var1 = ((((adc_T>>3) - ((int32_t)BPS_Dig_T1<<1))) * ((int32_t)BPS_Dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)BPS_Dig_T1)) * ((adc_T>>4) - ((int32_t)BPS_Dig_T1)))>> 12) *((int32_t)BPS_Dig_T3)) >> 14;
	BPS_BPS_Temperature_Fina = var1 + var2;
	T = (BPS_BPS_Temperature_Fina * 5 + 128) >> 8;
	return T;
}

void BPS_WakeUP(void)
{
	uint8_t datatowrite = 0;

	// first read the register
	HAL_I2C_Mem_Read(&BME280_port, BME280_ADDR, CTRL_MEAS_REG, 1, &datatowrite, 1, I2C_TimeOut);

	// modify the data with the forced mode
	datatowrite = datatowrite | MODE_FORCED;

	// write the new data to the register
	HAL_I2C_Mem_Write(&BME280_port, BME280_ADDR, CTRL_MEAS_REG, 1, &datatowrite, 1, I2C_TimeOut);

	HAL_Delay (100);
}

void BPS_Calibracao(void)
{
	uint8_t trimdata[32];

	HAL_I2C_Mem_Read(&BME280_port, BME280_ADDR, 0x88, 1, trimdata, 25, HAL_MAX_DELAY);

	HAL_I2C_Mem_Read(&BME280_port, BME280_ADDR, 0xE1, 1, (uint8_t *)trimdata+25, 7, HAL_MAX_DELAY);

	BPS_Dig_T1 = (trimdata[1]<<8) | trimdata[0];
	BPS_Dig_T2 = (trimdata[3]<<8) | trimdata[2];
	BPS_Dig_T3 = (trimdata[5]<<8) | trimdata[4];
	BPS_Dig_P1 = (trimdata[7]<<8) | trimdata[5];
	BPS_Dig_P2 = (trimdata[9]<<8) | trimdata[6];
	BPS_Dig_P3 = (trimdata[11]<<8) | trimdata[10];
	BPS_Dig_P4 = (trimdata[13]<<8) | trimdata[12];
	BPS_Dig_P5 = (trimdata[15]<<8) | trimdata[14];
	BPS_Dig_P6 = (trimdata[17]<<8) | trimdata[16];
	BPS_Dig_P7 = (trimdata[19]<<8) | trimdata[18];
	BPS_Dig_P8 = (trimdata[21]<<8) | trimdata[20];
	BPS_Dig_P9 = (trimdata[23]<<8) | trimdata[22];
	BPS_Dig_H1 = trimdata[24];
	BPS_Dig_H2 = (trimdata[26]<<8) | trimdata[25];
	BPS_Dig_H3 = (trimdata[27]);
	BPS_Dig_H4 = (trimdata[28]<<4) | (trimdata[29] & 0x0f);
	BPS_Dig_H5 = (trimdata[30]<<4) | (trimdata[29]>>4);
	BPS_Dig_H6 = (trimdata[31]);
}
#endif

#if (INA_definido == 1)
void INA219_Reset(){
	uint8_t addr[2];
	addr[0] = (INA219_CONFIG_RESET >> 8) & 0xff;  // upper byte
	addr[1] = (INA219_CONFIG_RESET >> 0) & 0xff; // lower byte
	HAL_I2C_Mem_Write(&INA219_port, INA219_ADDR, INA219_REG_CONFIG, 1, (uint8_t*)addr, 2, I2C_TimeOut);
	HAL_Delay(100);
}

void INA219_Calibracao(){
	uint16_t config_INA = INA219_CONFIG_BVOLTAGERANGE_16V |
	                    INA219_CONFIG_GAIN_1_40MV | INA219_CONFIG_BADCRES_12BIT |
	                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
	                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

	uint8_t addr1[2];
	addr1[0] = (ina219_calibrationValue >> 8) & 0xff;  // upper byte
	addr1[1] = (ina219_calibrationValue >> 0) & 0xff; // lower byte
	HAL_I2C_Mem_Write(&INA219_port, INA219_ADDR, INA219_REG_CALIBRATION, 1, (uint8_t*)addr1, 2, I2C_TimeOut);

	uint8_t addr2[2];
	addr2[0] = (config_INA >> 8) & 0xff;  // upper byte
	addr2[1] = (config_INA >> 0) & 0xff; // lower byte
	HAL_I2C_Mem_Write(&INA219_port, INA219_ADDR, INA219_REG_CONFIG, 1, (uint8_t*)addr2, 2, I2C_TimeOut);
}
#endif

#if (IMU_definido == 1)
void IMU_Accel_Read_RAW(void) {

	HAL_I2C_Mem_Read(&MPU6050_port, MPU6050_ADDR, MPU6050_ACCEL_REG_ADDR, 1, (uint8_t*)I2C1_RegsBuffer, 6, I2C_TimeOut);

	IMU_Acc_X_RAW_VET[0] = (int16_t)(I2C1_RegsBuffer[0] << 8 | I2C1_RegsBuffer[1]);
	IMU_Acc_Y_RAW_VET[0] = (int16_t)(I2C1_RegsBuffer[2] << 8 | I2C1_RegsBuffer[3]);
	IMU_Acc_Z_RAW_VET[0] = (int16_t)(I2C1_RegsBuffer[4] << 8 | I2C1_RegsBuffer[5]);

	memset((uint8_t*)I2C1_RegsBuffer, 0, I2C_Buffer_Size+1);
}

void IMU_Temp_Read_RAW(void) {

	HAL_I2C_Mem_Read(&MPU6050_port, MPU6050_ADDR, MPU6050_TEMP_REG_ADDR, 1, (uint8_t*)I2C1_RegsBuffer, 2, I2C_TimeOut);

	IMU_Temp_RAW_VET[Contador_Sensor_Index] = (int16_t)(I2C1_RegsBuffer[0] << 8 | I2C1_RegsBuffer[1]);

	memset((uint8_t*)I2C1_RegsBuffer, 0, I2C_Buffer_Size+1);
}

void IMU_Gyro_Read_RAW(void) {

	HAL_I2C_Mem_Read(&MPU6050_port, MPU6050_ADDR, MPU6050_GYRO_REG_ADDR, 1, (uint8_t*)I2C1_RegsBuffer, 6, HAL_MAX_DELAY);

	IMU_Gyro_X_RAW_VET[0] = (int16_t)(I2C1_RegsBuffer[0] << 8 | I2C1_RegsBuffer[1]);
	IMU_Gyro_Y_RAW_VET[0] = (int16_t)(I2C1_RegsBuffer[2] << 8 | I2C1_RegsBuffer[3]);
	IMU_Gyro_Z_RAW_VET[0] = (int16_t)(I2C1_RegsBuffer[4] << 8 | I2C1_RegsBuffer[5]);

	memset((uint8_t*)I2C1_RegsBuffer, 0, I2C_Buffer_Size+1);
}
#endif

#if (BPS_definido == 1)
void BPS_Read_RAW(void){

	HAL_I2C_Mem_Read(&BME280_port, BME280_ADDR, PRESS_MSB_REG, 1, (uint8_t*)I2C1_RegsBuffer, 8, HAL_MAX_DELAY);

	BPS_Pressao_RAW_VET[Contador_Sensor_Index] = (I2C1_RegsBuffer[0]<<12)|(I2C1_RegsBuffer[1]<<4)|(I2C1_RegsBuffer[2]>>4);
	BPS_Temperatura_RAW_VET[Contador_Sensor_Index] = (I2C1_RegsBuffer[3]<<12)|(I2C1_RegsBuffer[4]<<4)|(I2C1_RegsBuffer[5]>>4);
	BPS_Umidade_RAW_VET[Contador_Sensor_Index] = (I2C1_RegsBuffer[6]<<8)|(I2C1_RegsBuffer[7]);

	memset((uint8_t*)I2C1_RegsBuffer, 0, I2C_Buffer_Size+1);
}
#endif

#if (INA_definido == 1)
void INA219_Read_RAW(void){

	HAL_I2C_Mem_Read(&INA219_port, INA219_ADDR, INA219_REG_BUSVOLTAGE, 1, (uint8_t*)I2C1_RegsBuffer, 2, HAL_MAX_DELAY);

	INA219_vBus_RAW_VET[Contador_Sensor_Index] = ((uint16_t)((I2C1_RegsBuffer[0] << 8) | I2C1_RegsBuffer[1]) >> 3);

	memset((uint8_t*)I2C1_RegsBuffer, 0, I2C_Buffer_Size+1);


	HAL_I2C_Mem_Read(&INA219_port, INA219_ADDR, INA219_REG_SHUNTVOLTAGE, 1, (uint8_t*)I2C1_RegsBuffer, 2, HAL_MAX_DELAY);

	INA219_vShunt_RAW_VET[Contador_Sensor_Index] = ((uint16_t)((I2C1_RegsBuffer[0] << 8) | I2C1_RegsBuffer[1]));

	memset((uint8_t*)I2C1_RegsBuffer, 0, I2C_Buffer_Size+1);


	HAL_I2C_Mem_Read(&INA219_port, INA219_ADDR, INA219_REG_CURRENT, 1, (uint8_t*)I2C1_RegsBuffer, 2, HAL_MAX_DELAY);

	INA219_Corrente_RAW_VET[Contador_Sensor_Index] = ((uint16_t)((I2C1_RegsBuffer[0] << 8) | I2C1_RegsBuffer[1]));

	memset((uint8_t*)I2C1_RegsBuffer, 0, I2C_Buffer_Size+1);

}
#endif

void IniciaEscuta_USART2(void){
#if (UART_IDLE_definido == 1)
	if((FLAG_UART_LIVRE == 1) || (FLAG_UART_BOOT == 1)){
		FLAG_UART_LIVRE = 0;
		FLAG_UART_BOOT = 0;
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t*)RXBufferUSART2, USART2_BUFFER_SIZE);
		__HAL_DMA_DISABLE_IT(&handle_GPDMA1_Channel10, DMA_IT_HT);
		__HAL_DMA_DISABLE_IT(&handle_GPDMA1_Channel10, DMA_IT_TC);
		//__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
	}
#endif
}

void GPS_Index(void) {
#if (UART_IDLE_definido == 1)
	if (RXBuffer_Index == 0){
		memcpy((uint8_t*)MainBufferUSART2_0, (uint8_t*)RXBufferUSART2, USART2_BUFFER_SIZE+1);
		memcpy((uint8_t*)ErrorBufferUSART2_0, (uint8_t*)RXBufferUSART2, USART2_BUFFER_SIZE+1);
		memset((uint8_t*)MainBufferUSART2_1, 0, USART2_BUFFER_SIZE+1);
		memset((uint8_t*)ErrorBufferUSART2_1, 0, USART2_BUFFER_SIZE+1);
		RXBuffer_Index = 1;
	} else if (RXBuffer_Index == 1){
		memcpy((uint8_t*)MainBufferUSART2_1, (uint8_t*)RXBufferUSART2, USART2_BUFFER_SIZE+1);
		memcpy((uint8_t*)ErrorBufferUSART2_1, (uint8_t*)RXBufferUSART2, USART2_BUFFER_SIZE+1);
		memset((uint8_t*)MainBufferUSART2_2, 0, USART2_BUFFER_SIZE+1);
		memset((uint8_t*)ErrorBufferUSART2_2, 0, USART2_BUFFER_SIZE+1);
		RXBuffer_Index = 2;
	} else if (RXBuffer_Index == 2){
		memcpy((uint8_t*)MainBufferUSART2_2, (uint8_t*)RXBufferUSART2, USART2_BUFFER_SIZE+1);
		memcpy((uint8_t*)ErrorBufferUSART2_2, (uint8_t*)RXBufferUSART2, USART2_BUFFER_SIZE+1);
		memset((uint8_t*)MainBufferUSART2_0, 0, USART2_BUFFER_SIZE+1);
		memset((uint8_t*)ErrorBufferUSART2_0, 0, USART2_BUFFER_SIZE+1);
		RXBuffer_Index = 0;
	}
	memset((uint8_t*)RXBufferUSART2, 0, USART2_BUFFER_SIZE+1);
#endif
}

#if (GPS_definido == 1)
int GPS_VALIDANDO_PARSE_XNMEA(char *STR_BUFFER, uint16_t STR_index) {
    char check[3];
    char checkcalcstr[3];
    uint16_t i = STR_index;
    uint16_t i_inicio = STR_index;
    uint8_t calculated_check = 0;
    uint8_t NMEA_PARAM = 0;
    uint16_t i_temp_PARSE = 0;
    /* NMEA_PARAM =
    NULL	0
	$GNRMC	1
	$GNVTG	2
	$GNGGA	3
	$GNGSA	4
	$GPGSV	5
	$GPGSV	6
	$GPGSV	7
	$GNGLL	8
	*/

    i = STR_index;
    while (((char)STR_BUFFER[i] != '$') && (i <= (USART2_BUFFER_SIZE-15))){
    	i++;
    }

	if (((char)STR_BUFFER[i] == '$') && ((char)STR_BUFFER[i+1] == 'G') && ((char)STR_BUFFER[i+2] == 'N') &&  ((char)STR_BUFFER[i+3] == 'R') &&  ((char)STR_BUFFER[i+4] == 'M') &&  ((char)STR_BUFFER[i+5] == 'C')){
		NMEA_PARAM = 1;
	} else if (((char)STR_BUFFER[i] == '$') && ((char)STR_BUFFER[i+1] == 'G') && ((char)STR_BUFFER[i+2] == 'N') &&  ((char)STR_BUFFER[i+3] == 'V') &&  ((char)STR_BUFFER[i+4] == 'T') &&  ((char)STR_BUFFER[i+5] == 'G')){
		NMEA_PARAM = 2;
	} else if (((char)STR_BUFFER[i] == '$') && ((char)STR_BUFFER[i+1] == 'G') && ((char)STR_BUFFER[i+2] == 'N') &&  ((char)STR_BUFFER[i+3] == 'G') &&  ((char)STR_BUFFER[i+4] == 'G') &&  ((char)STR_BUFFER[i+5] == 'A')){
		NMEA_PARAM = 3;
	} else if (((char)STR_BUFFER[i] == '$') && ((char)STR_BUFFER[i+1] == 'G') && ((char)STR_BUFFER[i+2] == 'N') &&  ((char)STR_BUFFER[i+3] == 'G') &&  ((char)STR_BUFFER[i+4] == 'S') &&  ((char)STR_BUFFER[i+5] == 'A')){
		NMEA_PARAM = 4;
	} else if (((char)STR_BUFFER[i] == '$') && ((char)STR_BUFFER[i+1] == 'G') && ((char)STR_BUFFER[i+2] == 'P') &&  ((char)STR_BUFFER[i+3] == 'G') &&  ((char)STR_BUFFER[i+4] == 'S') &&  ((char)STR_BUFFER[i+5] == 'V')){
		NMEA_PARAM = 5;
	} else if (((char)STR_BUFFER[i] == '$') && ((char)STR_BUFFER[i+1] == 'G') && ((char)STR_BUFFER[i+2] == 'N') &&  ((char)STR_BUFFER[i+3] == 'G') &&  ((char)STR_BUFFER[i+4] == 'L') &&  ((char)STR_BUFFER[i+5] == 'L')){
		NMEA_PARAM = 6;
	} else if ((i >= (USART2_BUFFER_SIZE-15))){
		FLAG_GPS_VALIDANDO = 0;
        return 0;
	} else {
		FLAG_GPS_VALIDANDO = 0;
        return 0;
	}

    while(((char)STR_BUFFER[i] != 0) && ((char)STR_BUFFER[i] != '*') && ((i - i_inicio) <= 77)){
        if (NMEA_PARAM == 1){
        	GPS_PARSE_GNRMC[i_temp_PARSE] = STR_BUFFER[i];
        } else if (NMEA_PARAM == 2){
        	GPS_PARSE_GNVTG[i_temp_PARSE] = STR_BUFFER[i];
        } else if (NMEA_PARAM == 3){
        	GPS_PARSE_GNGGA[i_temp_PARSE] = STR_BUFFER[i];
        } else if (NMEA_PARAM == 4){
        	GPS_PARSE_GNGSA[i_temp_PARSE] = STR_BUFFER[i];
        } else if (NMEA_PARAM == 5){
        	GPS_PARSE_GPGSV[i_temp_PARSE] = STR_BUFFER[i];
        } else if (NMEA_PARAM == 6) {
        	GPS_PARSE_GNGLL[i_temp_PARSE] = STR_BUFFER[i];
        }
        if (STR_BUFFER[i] != '$'){
        	calculated_check ^= (char)STR_BUFFER[i];
        }
        i++;
        i_temp_PARSE++;
    }

    if (((char)STR_BUFFER[i] == '*') && (i <= (USART2_BUFFER_SIZE-2))){
        check[0] = (char)STR_BUFFER[i+1];
        check[1] = (char)STR_BUFFER[i+2];
        check[2] = 0;
    } else {
    	FLAG_GPS_VALIDANDO = 0;
        return 0;
    }

    sprintf(checkcalcstr,"%02X",calculated_check);

    if ((checkcalcstr[0] == check[0]) && (checkcalcstr[1] == check[1])){
        if (NMEA_PARAM == 1){
        	sprintf((char*)GPS_CHECKSUM_GNRMC,"%02X",calculated_check);
        } else if (NMEA_PARAM == 2){
        	sprintf((char*)GPS_CHECKSUM_GNVTG,"%02X",calculated_check);
        } else if (NMEA_PARAM == 3){
        	sprintf((char*)GPS_CHECKSUM_GNGGA,"%02X",calculated_check);
        } else if (NMEA_PARAM == 4){
        	sprintf((char*)GPS_CHECKSUM_GNGSA,"%02X",calculated_check);
        } else if (NMEA_PARAM == 5){
        	sprintf((char*)GPS_CHECKSUM_GPGSV,"%02X",calculated_check);
        } else if (NMEA_PARAM == 6) {
        	FLAG_GPS_VALIDANDO = 0;
        	sprintf((char*)GPS_CHECKSUM_GNGLL,"%02X",calculated_check);
        }
        return i;
    } else {
    	FLAG_GPS_VALIDANDO = 0;
    	return 0;
    }
}
#endif

#if (GPS_definido == 1)
void GPS_parse(){
    if(!strncmp((char*)GPS_PARSE_GNGGA, "$GNGGA", 6)){
    	if (sscanf((char*)GPS_PARSE_GNGGA, "$GNGGA,%f,%f,%c,%f,%c,%d,%d,%f,%f,%c", &GPS.utc_time, &GPS.nmea_latitude, &GPS.ns, &GPS.nmea_longitude, &GPS.ew, &GPS.lock, &GPS.satelites, &GPS.hdop, &GPS.msl_altitude, &GPS.msl_units) >= 1){
    		GPS.dec_latitude = GPS_nmea_to_dec(GPS.nmea_latitude, GPS.ns);
    		GPS.dec_longitude = GPS_nmea_to_dec(GPS.nmea_longitude, GPS.ew);
    	}
    }
    if (!strncmp((char*)GPS_PARSE_GNRMC, "$GNRMC", 6)){
    	if(sscanf((char*)GPS_PARSE_GNRMC, "$GNRMC,%f,%f,%c,%f,%c,%f,%f,%d", &GPS.utc_time, &GPS.nmea_latitude, &GPS.ns, &GPS.nmea_longitude, &GPS.ew, &GPS.speed_k, &GPS.course_d, &GPS.date) >= 1){

    	}

    }
    if (!strncmp((char*)GPS_PARSE_GNGLL, "$GNGLL", 6)){
        if(sscanf((char*)GPS_PARSE_GNGLL, "$GNGLL,%f,%c,%f,%c,%f,%c", &GPS.nmea_latitude, &GPS.ns, &GPS.nmea_longitude, &GPS.ew, &GPS.utc_time, &GPS.gll_status) >= 1){

        }
    }
    if (!strncmp((char*)GPS_PARSE_GNVTG, "$GNVTG", 6)){
        if(sscanf((char*)GPS_PARSE_GNVTG, "$GNVTG,%f,%c,%f,%c,%f,%c,%f,%c", &GPS.course_t, &GPS.course_t_unit, &GPS.course_m, &GPS.course_m_unit, &GPS.speed_k, &GPS.speed_k_unit, &GPS.speed_km, &GPS.speed_km_unit) >= 1){

        }
    }
}
#endif

#if (GPS_definido == 1)
void memset0_Buff_GPS() {

    memset((uint8_t*)GPS_PARSE_GNRMC, 0, NMEA_BUFFER_SIZE+1);
    memset((char*)GPS_CHECKSUM_GNRMC, 0, NMEA_BUFFER_CKS_SIZE+1);

    memset((uint8_t*)GPS_PARSE_GNVTG, 0, NMEA_BUFFER_SIZE+1);
    memset((char*)GPS_CHECKSUM_GNVTG, 0, NMEA_BUFFER_CKS_SIZE+1);

    memset((uint8_t*)GPS_PARSE_GNGGA, 0, NMEA_BUFFER_SIZE+1);
    memset((char*)GPS_CHECKSUM_GNGGA, 0, NMEA_BUFFER_CKS_SIZE+1);

    memset((uint8_t*)GPS_PARSE_GNGSA, 0, NMEA_BUFFER_SIZE+1);
    memset((char*)GPS_CHECKSUM_GNGSA, 0, NMEA_BUFFER_CKS_SIZE+1);

    memset((uint8_t*)GPS_PARSE_GPGSV, 0, NMEA_BUFFER_SIZE+1);
    memset((char*)GPS_CHECKSUM_GPGSV, 0, NMEA_BUFFER_CKS_SIZE+1);

    memset((uint8_t*)GPS_PARSE_GNGLL, 0, NMEA_BUFFER_SIZE+1);
    memset((char*)GPS_CHECKSUM_GNGLL, 0, NMEA_BUFFER_CKS_SIZE+1);

    memset((uint8_t*)GPS_PARSE_GNGLL, 0, NMEA_BUFFER_SIZE+1);

}
#endif

void memset0_Buff_Sensores() {
#if (IMU_definido == 1)
	IMU_Ax = 0;
	IMU_Ay = 0;
	IMU_Az = 0;
	IMU_T = 0;
	IMU_Gx = 0;
	IMU_Gy = 0;
	IMU_Gz = 0;
#endif

#if (BPS_definido == 1)
	BPS_Temperatura = 0;
	BPS_Pressao = 0;
	BPS_Umidade = 0;
	BPS_Altitude = 0;
#endif

#if (INA_definido == 1)
	INA219_vBus = 0;
	INA219_vShunt = 0;
	INA219_Corrente = 0;
#endif

/*
#if (GPS_definido == 1)
	GPS.nmea_latitude = 0;
	GPS.ns = 'S';
	GPS.nmea_longitude = 0;
	GPS.ew = 'W';
	GPS.dec_latitude = 0;
	GPS.dec_longitude = 0;
	GPS.msl_altitude = 0;
	GPS.utc_time = 0;
	GPS.satelites = 0;
#endif
*/
}

float GPS_nmea_to_dec(float deg_coord, char nsew) {
    int degree = (int)(deg_coord/100);
    float minutes = deg_coord - degree*100;
    float dec_deg = minutes / 60;
    float decimal = degree + dec_deg;
    if (nsew == 'S' || nsew == 'W') { // return negative
        decimal *= -1;
    }
    return decimal;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){

#if (UART_IDLE_definido == 1)
	if (huart->Instance == USART2){
		FLAG_UART_LIVRE = 0;
		HAL_UART_DMAStop(&huart2);
		//HAL_UART_DMAPause(huart);
		//HAL_UART_Abort(huart);
		//HAL_UART_AbortReceive(huart);
		//HAL_UART_Abort_IT(huart);
		//HAL_UART_AbortReceive_IT(huart);
		GPS_Index();
		FLAG_UART_LIVRE = 1;
	}
#endif

}

//20hz
void IT_TIM6_CALLBACK() {

	Contador_TIM6++;
	Contador_TIM6_1hz++;
	if (FLAG_SD_PROCESSANDO == 0){
		Contador_SENSOR++;

#if (IMU_definido == 1)
			IMU_Accel_Read_RAW();
			IMU_Temp_Read_RAW();
			IMU_Gyro_Read_RAW();
#endif

#if (BPS_definido == 1)
			BPS_Read_RAW();
#endif

#if (INA_definido == 1)
			INA219_Read_RAW();
#endif

		Contador_Sensor_Index++;

		if(Contador_TIM6_1hz == 1){
			FLAG_SD_PROCESSANDO = 1;
			for (int i = 0; i <= Contador_Sensor_Index; i++){
#if (IMU_definido == 1)
				IMU_Acc_X_RAW += IMU_Acc_X_RAW_VET[i];
				IMU_Acc_Y_RAW += IMU_Acc_Y_RAW_VET[i];
				IMU_Acc_Z_RAW += IMU_Acc_Z_RAW_VET[i];
				IMU_Temp_RAW += IMU_Temp_RAW_VET[i];
				IMU_Gyro_X_RAW += IMU_Gyro_X_RAW_VET[i];
				IMU_Gyro_Y_RAW += IMU_Gyro_Y_RAW_VET[i];
				IMU_Gyro_Z_RAW += IMU_Gyro_Z_RAW_VET[i];
#endif

#if (BPS_definido == 1)
				BPS_Pressao_RAW += BPS_Pressao_RAW_VET[i];
				BPS_Temperatura_RAW += BPS_Temperatura_RAW_VET[i];
				BPS_Umidade_RAW += BPS_Umidade_RAW_VET[i];
#endif

#if (INA_definido == 1)
				INA219_vBus_RAW += INA219_vBus_RAW_VET[i];
				INA219_vShunt_RAW += INA219_vShunt_RAW_VET[i];
				INA219_Corrente_RAW += INA219_Corrente_RAW_VET[i];
#endif
			}

#if (IMU_definido == 1)
			IMU_Ax = ((float) (IMU_Acc_X_RAW_VET[0]) *10 / 16384.0);
			IMU_Ay = ((float) (IMU_Acc_Y_RAW_VET[0]) *10 / 16384.0);
			IMU_Az = ((float) (IMU_Acc_Z_RAW_VET[0]) *10 / 16384.0);

			//IMU_Ax = ((float) (IMU_Acc_X_RAW/Contador_Sensor_Index) *10 / 16384.0);
			//IMU_Ay = ((float) (IMU_Acc_Y_RAW/Contador_Sensor_Index) *10 / 16384.0);
			//IMU_Az = ((float) (IMU_Acc_Z_RAW/Contador_Sensor_Index) *10 / 16384.0);
			memset((int16_t*)IMU_Acc_X_RAW_VET, 0, SENSOR_Tam_Buffer+1);
			memset((int16_t*)IMU_Acc_Y_RAW_VET, 0, SENSOR_Tam_Buffer+1);
			memset((int16_t*)IMU_Acc_Z_RAW_VET, 0, SENSOR_Tam_Buffer+1);
			IMU_Acc_X_RAW = 0;
			IMU_Acc_Y_RAW = 0;
			IMU_Acc_Z_RAW = 0;

			IMU_T = ((IMU_Temp_RAW/Contador_Sensor_Index)/340) + 36.53;
			memset((int16_t*)IMU_Temp_RAW_VET, 0, SENSOR_Tam_Buffer+1);
			IMU_Temp_RAW = 0;

			IMU_Gx = ((float) (IMU_Gyro_X_RAW_VET[0]) / 500.0);
			IMU_Gy = ((float) (IMU_Gyro_Y_RAW_VET[0]) / 500.0);
			IMU_Gz = ((float) (IMU_Gyro_Z_RAW_VET[0]) / 500.0);

			//IMU_Gx = ((float) (IMU_Gyro_X_RAW/Contador_Sensor_Index) / 500.0);
			//IMU_Gy = ((float) (IMU_Gyro_Y_RAW/Contador_Sensor_Index) / 500.0);
			//IMU_Gz = ((float) (IMU_Gyro_Z_RAW/Contador_Sensor_Index) / 500.0);
			memset((int16_t*)IMU_Gyro_X_RAW_VET, 0, SENSOR_Tam_Buffer+1);
			memset((int16_t*)IMU_Gyro_Y_RAW_VET, 0, SENSOR_Tam_Buffer+1);
			memset((int16_t*)IMU_Gyro_Z_RAW_VET, 0, SENSOR_Tam_Buffer+1);
			IMU_Gyro_X_RAW = 0;
			IMU_Gyro_Y_RAW = 0;
			IMU_Gyro_Z_RAW = 0;
#endif

#if (BPS_definido == 1)
			BPS_Pressao = (BPS_Pressao_Comp(BPS_Pressao_RAW/Contador_Sensor_Index))/256.0;
			BPS_Temperatura = (BPS_Temperatura_Comp(BPS_Temperatura_RAW/Contador_Sensor_Index))/100.0;
			BPS_Umidade = (BPS_Umidade_Comp(BPS_Umidade_RAW/Contador_Sensor_Index))/1024.0;
			double BPS_TemperaturaK = BPS_Temperatura + 273.15;
			double temp1 = BPS_TemperaturaK/BPS_Lb;
			double temp2 = BPS_Pressao/101325;
			BPS_Altitude = temp1 * ((pow(temp2, BPS_Const)) - 1);
			memset((int32_t*)BPS_Pressao_RAW_VET, 0, SENSOR_Tam_Buffer+1);
			memset((int32_t*)BPS_Temperatura_RAW_VET, 0, SENSOR_Tam_Buffer+1);
			memset((int32_t*)BPS_Umidade_RAW_VET, 0, SENSOR_Tam_Buffer+1);
			BPS_Pressao_RAW = 0;
			BPS_Temperatura_RAW = 0;
			BPS_Umidade_RAW = 0;
#endif

#if (INA_definido == 1)
			INA219_vBus = ((INA219_vBus_RAW/Contador_Sensor_Index) * 4);
			INA219_vShunt = ((INA219_vShunt_RAW/Contador_Sensor_Index) * 0.01);
			INA219_Corrente = ((INA219_Corrente_RAW/Contador_Sensor_Index) / ina219_currentDivider_mA);
			INA219_vBus = INA219_vBus/1000;
			INA219_vShunt = INA219_vShunt/1000;
			INA219_Corrente = INA219_Corrente/1000;
			memset((uint16_t*)INA219_vBus_RAW_VET, 0, SENSOR_Tam_Buffer+1);
			memset((uint16_t*)INA219_vShunt_RAW_VET, 0, SENSOR_Tam_Buffer+1);
			memset((uint16_t*)INA219_Corrente_RAW_VET, 0, SENSOR_Tam_Buffer+1);
			INA219_vBus_RAW = 0;
			INA219_vShunt_RAW = 0;
			INA219_Corrente_RAW = 0;

#endif

			Contador_Sensor_Index = 0;
			Contador_TIM6_1hz = 0;

#if (GPS_definido == 1)
			uint16_t i_retorno = 0;

			FLAG_GPS_VALIDANDO = 1;

#if (UART_IDLE_definido == 1)
			if (RXBuffer_Index == 0){
				while ((i_retorno <= (USART2_BUFFER_SIZE-10)) && (FLAG_GPS_VALIDANDO == 1)){
					i_retorno = GPS_VALIDANDO_PARSE_XNMEA((char*)MainBufferUSART2_2, i_retorno);
				}
			} else if (RXBuffer_Index == 1){
				while ((i_retorno <= (USART2_BUFFER_SIZE-10)) && (FLAG_GPS_VALIDANDO == 1)){
					i_retorno = GPS_VALIDANDO_PARSE_XNMEA((char*)MainBufferUSART2_0, i_retorno);
				}
			} else if (RXBuffer_Index == 2){
				while ((i_retorno <= (USART2_BUFFER_SIZE-10)) && (FLAG_GPS_VALIDANDO == 1)){
					i_retorno = GPS_VALIDANDO_PARSE_XNMEA((char*)MainBufferUSART2_1, i_retorno);
				}
			}
#endif

			if ((FLAG_GPS_VALIDANDO == 0) && (i_retorno != 0)) {
				GPS_parse();
				memset0_Buff_GPS();
				if ((GPS.nmea_latitude != 0) && (GPS.dec_longitude != 0) && (FLAG_LED_200ms == 0)){
					FLAG_LED_50ms++;
				} else if ((GPS.nmea_latitude == 0) && (GPS.dec_longitude == 0)) {
					if (FLAG_LED_50ms == 0){
						FLAG_LED_200ms++;
					}
					FLAG_GPS_SD_ERROR = 1;
					Contador_GPS_ERRO++;
				}
			}
#endif

#if (SD_definido == 1)
			HAL_RTC_GetTime(&hrtc, &RTC_tempo, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &RTC_data, RTC_FORMAT_BIN);

			RTC_Segundos = RTC_tempo.Seconds;
			RTC_Minutos = RTC_tempo.Minutes;
			RTC_Horas = RTC_tempo.Hours;
			SD_LOG_rotina();
			FLAG_SD_PROCESSANDO = 0;
			Contador_SD_ROTINA++;
			memset0_Buff_Sensores();
#endif
		}
	}
#if (UART_IDLE_definido == 1)
	IniciaEscuta_USART2();
#endif
}

//5hz
void IT_TIM7_CALLBACK() {

	Contador_TIM7++;
}

//5hz
void IT_TIM16_CALLBACK() {

	Contador_TIM16++;
}

//100hz
void IT_TIM17_CALLBACK() {

	Contador_TIM17++;

	if (FLAG_LED_200ms > 0){
		HAL_GPIO_WritePin(LED_DEBUG_PORT, LED_DEBUG_PIN, GPIO_PIN_SET);
		Contador_TIM17_200ms++;
		if(Contador_TIM17_200ms > 20){
			HAL_GPIO_WritePin(LED_DEBUG_PORT, LED_DEBUG_PIN, GPIO_PIN_RESET);
			Contador_TIM17_200ms = 0;
			FLAG_LED_200ms--;
		}
	}

	if (FLAG_LED_50ms > 0){
		HAL_GPIO_WritePin(LED_DEBUG_PORT, LED_DEBUG_PIN, GPIO_PIN_SET);
		Contador_TIM17_50ms++;
		if(Contador_TIM17_50ms > 2){
			HAL_GPIO_WritePin(LED_DEBUG_PORT, LED_DEBUG_PIN, GPIO_PIN_RESET);
			Contador_TIM17_50ms = 0;
			FLAG_LED_50ms--;
		}
	}
}

void SD_Error_Handler(void){
  __disable_irq();
  while (1)
  {
	  HAL_GPIO_WritePin(LED_DEBUG_PORT, LED_DEBUG_PIN, GPIO_PIN_RESET);

#if (SD_definido == 1)
	  /* fecha montagem  */
	  fx_media_close(&sdio_disk);
#endif

	  NVIC_SystemReset();

  }
}

void UART2_Error_Handler(void){
  __disable_irq();
  while (1)
  {
	  HAL_GPIO_WritePin(LED_DEBUG_PORT, LED_DEBUG_PIN, GPIO_PIN_RESET);

#if (SD_definido == 1)
	  /* fecha montagem  */
	  fx_media_close(&sdio_disk);
#endif

	  NVIC_SystemReset();

  }
}

void GPS_Error_Handler(void){
	  //__disable_irq();
	  HAL_GPIO_WritePin(LED_DEBUG_PORT, LED_DEBUG_PIN, GPIO_PIN_RESET);

	#if (SD_definido == 1)
		  /* fecha montagem  */
	  //fx_media_close(&sdio_disk);
	#endif

	  //NVIC_SystemReset();
}

void IMU_Error_Handler(void){
	  //__disable_irq();
	  HAL_GPIO_WritePin(LED_DEBUG_PORT, LED_DEBUG_PIN, GPIO_PIN_RESET);

	#if (SD_definido == 1)
		  /* fecha montagem  */
	  //fx_media_close(&sdio_disk);
	#endif

	  //NVIC_SystemReset();
}

void BPS_Error_Handler(void){
	  //__disable_irq();
	  HAL_GPIO_WritePin(LED_DEBUG_PORT, LED_DEBUG_PIN, GPIO_PIN_RESET);

	#if (SD_definido == 1)
		  /* fecha montagem  */
	  //fx_media_close(&sdio_disk);
	#endif

	  //NVIC_SystemReset();
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
	  HAL_GPIO_WritePin(LED_DEBUG_PORT, LED_DEBUG_PIN, GPIO_PIN_RESET);

#if (SD_definido == 1)
	  /* fecha montagem  */
	  fx_media_close(&sdio_disk);
#endif

	  NVIC_SystemReset();
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
