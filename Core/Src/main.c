/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdlib.h>
#include "am2320.h"
#include <math.h>
#include <stdio.h>
#include "MPU9250.h"
#include "ms5607.h"

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

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/*
 * I2C1 - temp/hum
 *
 * SPI1 - 9DOF
 * SPI2 - MS5607
 *
 * huart1 - GNSS
 * huart2 - USB
 * huart3 - SD
 * huart4 - x
 * huart5 - tRF
 * huart6 - x
 */
uint16_t DataRecieved = 0; //delka prijatych zprav
uint8_t GPSbuffer[600] = {};
uint8_t GPSbuffer1[600] = {};
HAL_StatusTypeDef hal_st1, hal_st2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_UART5_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Reverses a string 'str' of length 'len'
//void parseRMC(uint8_t in[][11], uint8_t size)

void reverse(char* str, int len)
{
    int i = 0, j = len - 1, temp;
    while (i < j) {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}

// Converts a given integer x to string str[].
// d is the number of digits required in the output.
// If d is more than the number of digits in x,
// then 0s are added at the beginning.

int intToStr(int x, char str[], int d)
{
    int i = 0;
    if(x<0)
        x = x*-1;
    while (x) {
        str[i++] = (x % 10) + '0';
        x = x / 10;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}

void ftoa(float n, char* res, int beforepoint, int afterpoint, char sign)
{
    // Extract integer part
    short isnegative = 0;
    if(n<0)
        isnegative = 1;

    int ipart = (int)n;

    // Extract floating part
    float fpart = n - (float)ipart;

    // convert integer part to string
    int i = intToStr(ipart, res, beforepoint);

    // check for display option after point
    if (afterpoint != 0) {
        res[i] = '.'; // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter
        // is needed to handle cases like 233.007
        fpart = fpart * pow(10, afterpoint);

        intToStr((int)fpart, res + i + 1, afterpoint);
        if(sign){
			if(isnegative)
				res[0] = '-';
			else
				res[0] = '+';
        }
    }
}

/*void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	HAL_UART_Receive_IT(&huart2, buff, sizeof(buff));
}*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	DataRecieved = Size;
	//HAL_UARTEx_ReceiveToIdle_IT(&huart2, buff, sizeof(buff));
	//hal_st1 = HAL_UARTEx_ReceiveToIdle_IT(&huart1, GPSbuffer, sizeof(GPSbuffer));

}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	uint32_t er = huart->ErrorCode;
}



/*void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	DataRecieved = 1;
}*/


void parseFrame(uint8_t* GPSparse, uint8_t* msg, uint8_t* framelen, uint8_t nframes, uint8_t maxlen, uint8_t linenumber){
    int parsepos = 0;
    for(int i = 0; i < nframes; i++){
      while(!((GPSparse[200*linenumber + parsepos] == ',') || (GPSparse[200*linenumber + parsepos] == '*'))){ //dokud se nenarazi na oddelovac ,
        msg[i*maxlen + framelen[i]] = GPSparse[200*linenumber + parsepos];
        framelen[i]++;
        parsepos++;
      }
      parsepos++;
    }
  parsepos = 0;
}

void mainParse(uint8_t* buffer, uint8_t* parse, uint8_t linelen, uint8_t* nsat){
	uint8_t k[15] = {};		//k-ty znak v n-tem radku
	uint16_t iter = 0;		//iteracni promenna
	uint8_t n = 0;			//n-ty radek

	while(iter < DataRecieved){
		if(!(buffer[iter] == '\n') || (buffer[iter] == '\r')){ //dokud nenastane konec radku
			parse[n*linelen +  k[n]] = buffer[iter];
			if((k[n] == 7) && (parse[n*linelen + 5] == 'V')){ //hledani GSV zpravy, 7. znak je pocet satelitu
				*nsat = parse[n*linelen + k[n]] - '0';
			}
			k[n]++; //posun o znak dopredu
		}
		else{
			parse[n*linelen + k[n]] = '\n'; //enter na konec radku
			n++; //novy radek
		}
		iter++; //posun v RAW zprave - GPSbuffer
	}
}

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_UART5_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */



  MPU9250_Init();

  Am2320_HandleTypeDef Am2320_;
  Am2320_ = am2320_Init(&hi2c1, AM2320_ADDRESS);

  MS5607StateTypeDef state;
  state = MS5607_Init(&hspi2, GPIOC, GPIO_PIN_3);
  int32_t MSpressure = 0;
  double MStemp = 0;

  float temperature, humidity;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int8_t newline = '\n';





  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); //R
  HAL_Delay(50);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); //G
  HAL_Delay(50);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);  //B
  HAL_Delay(50);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);  //Y
  HAL_Delay(50);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);  //O
  HAL_Delay(50);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);





  hal_st1 = HAL_UARTEx_ReceiveToIdle_IT(&huart1, GPSbuffer, sizeof(GPSbuffer)); //TODO - why is calling it twice needed?
  hal_st1 = HAL_UARTEx_ReceiveToIdle_IT(&huart1, GPSbuffer, sizeof(GPSbuffer));





    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */



		if(DataRecieved){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); //R
			uint8_t GPSparse[15][200] = {}; //NMEA rozdelena podle radku
			uint8_t nsat = 0; //pocet GSV zprav

			mainParse(GPSbuffer, GPSparse, 200, &nsat);

	//PARSOVANI JEDNOTLIVYCH FRAMU

			uint8_t RMCposition[13] = {}; //delka daneho pole v RMC
			uint8_t RMC[13][11] = {}; //RMC buffer, GPS na 3,4,5,6

			uint8_t VTGposition[10] = {}; //delka daneho pole v RMC
			uint8_t VTG[10][10] = {}; 	//VTG buffer

			uint8_t GGAposition[15] = {}; //delka daneho pole v RMC
			uint8_t GGA[15][11] = {};	//GGA buffer

			uint8_t GSAposition[18] = {}; //delka daneho pole v RMC
			uint8_t GSA[18][5] = {}; //GSA buffer

			uint8_t GLLposition[8] = {}; //delka daneho pole v RMC
			uint8_t GLL[8][11] = {}; //GLL buffer

			parseFrame(GPSparse, RMC, RMCposition, 13, 11, 0);
			parseFrame(GPSparse, VTG, VTGposition, 10, 10, 1);
			parseFrame(GPSparse, GGA, GGAposition, 15, 11, 2);
			parseFrame(GPSparse, GSA, GSAposition, 18, 5, 3);
			parseFrame(GPSparse, GLL, GLLposition, 8, 11, 4+nsat);


		//PARSOVANI GSV
			//TODO
			//uint8_t GSV[3][17][15] = {}; //GSV buffer



		//Cteni teploty
			am2320_GetTemperatureAndHumidity(&Am2320_, &temperature, &humidity);
		//Cteni z Gyra
			int16_t AccData[3] = {}, GyroData[3] = {}, MagData[3] = {};
			MPU9250_GetData(AccData, GyroData, MagData);
			char AccStr[3][5]={}, GyroStr[3][5]={}, MagStr[3][5]={};

			for(int i = 0; i < 3; i++){
				itoa(AccData[i], AccStr[i], 10);
			}

			for(int i = 0; i < 3; i++){
				itoa(GyroData[i], GyroStr[i], 10);
			}

			for(int i = 0; i < 3; i++){
				itoa(MagData[i], MagStr[i], 10);
			}
		  //Teplota Tlak
			MS5607Update();
			HAL_Delay(10);
			MSpressure = MS5607GetPressurePa();
			MStemp = MS5607GetTemperatureC();


		//Sesiti danych poli do RX zprávy
			uint8_t RXbuffer[200] = {[0 ... 199] = ';'}; //Buffer na radiovy spoj UART3
			int cpypos = 0;

			//cas
			memcpy(RXbuffer + cpypos, RMC[1], RMCposition[1]);
			cpypos = cpypos + RMCposition[1] + 1;

			//souradnice
			for(int i = 3; i <= 6; i++){
				memcpy(RXbuffer + cpypos, RMC[i], RMCposition[i]);
				cpypos = cpypos + RMCposition[i] + 1;
			}

			//nadmorska vyska
			memcpy(RXbuffer + cpypos, GGA[9], GGAposition[9]);
			cpypos = cpypos + GGAposition[9] + 1;

			//rychlost (kmph)
			memcpy(RXbuffer + cpypos, VTG[7], VTGposition[7]);
			cpypos = cpypos + VTGposition[7] + 1;


			char tempchar[5];
			char humchar[5];
			ftoa(temperature, tempchar, 3, 1, 1); //pocet cislic pred teckou, za teckou a znamenko - 0/1
			ftoa(humidity, humchar, 3, 1, 0);

			char MStempchar[7];
			char MSpressurechar[7];

			ftoa((float)MStemp, MStempchar, 4, 2, 1);
			ftoa(MSpressure, MSpressurechar, 7, 0, 1);

			memcpy(RXbuffer + cpypos, tempchar, sizeof(tempchar));
			cpypos = cpypos+sizeof(tempchar) + 1;

			memcpy(RXbuffer + cpypos, humchar, sizeof(humchar));
			cpypos = cpypos+sizeof(humchar) + 1;

			memcpy(RXbuffer + cpypos, MStempchar, sizeof(MStempchar));
			cpypos = cpypos+sizeof(MStempchar) + 1;

			memcpy(RXbuffer + cpypos, MSpressurechar, sizeof(MSpressurechar));
			cpypos = cpypos+sizeof(MSpressurechar) + 1;

			for(int i = 0; i<3; i++){
				memcpy(RXbuffer + cpypos, AccStr[i], sizeof(AccStr[i]));
				cpypos = cpypos + sizeof(AccStr[i]) + 1;
			}
			cpypos++;

			for(int i = 0; i<3; i++){
				memcpy(RXbuffer + cpypos, GyroStr[i], sizeof(GyroStr[i]));
				cpypos = cpypos + sizeof(GyroStr[i]) + 1;
			}
			cpypos++;

			for(int i = 0; i<3; i++){
				memcpy(RXbuffer + cpypos, MagStr[i], sizeof(MagStr[i]));
				cpypos = cpypos + sizeof(MagStr[i]) + 1;
			}
			cpypos++;


			memcpy(RXbuffer + cpypos, &newline, sizeof(newline));

			HAL_UART_Transmit(&huart5, RXbuffer, cpypos+1, HAL_MAX_DELAY);
			HAL_UART_Transmit(&huart5, &newline, 1, HAL_MAX_DELAY);

		//Sesiti danych poli do SD zpravy
			uint8_t SDbuffer[300] = {[0 ... 298] = ';', '\n'}; //Buffer pro ukladani dat na SD kartu
			memcpy(SDbuffer, RXbuffer, cpypos); //kopirovani RX bufferu

			//fix type
			memcpy(SDbuffer + cpypos, GGA[6], GGAposition[6]);
			cpypos = cpypos + GGAposition[6] + 1;

			//pocet satelitu
			memcpy(SDbuffer + cpypos, GGA[7], GGAposition[7]);
			cpypos = cpypos + GGAposition[7] + 1;

			//PDOP
			memcpy(SDbuffer + cpypos, GSA[15], GSAposition[15]);
			cpypos = cpypos + GSAposition[15] + 1;
			//HDOP
			memcpy(SDbuffer + cpypos, GSA[16], GSAposition[16]);
			cpypos = cpypos + GSAposition[16] + 1;
			//VDOP
			memcpy(SDbuffer + cpypos, GSA[17], GSAposition[17]);
			cpypos = cpypos + GSAposition[17] + 1;

			memcpy(SDbuffer + cpypos, &newline, sizeof(newline));
			cpypos = cpypos + 2;

			HAL_UART_Transmit(&huart3, SDbuffer, cpypos-1, HAL_MAX_DELAY);

			//Cisteni
			memset(RXbuffer, 0, sizeof(RXbuffer));
			memset(GPSbuffer, 0, sizeof(GPSbuffer));
			DataRecieved = 0;

			//hal_st1 = HAL_UARTEx_ReceiveToIdle_IT(&huart1, GPSbuffer, sizeof(GPSbuffer)); //init ToIdle interruptu
			//HAL_UARTEx_ReceiveToIdle_IT(&huart1, GPSbuffer, sizeof(GPSbuffer));

			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

			//HAL_UARTEx_ReceiveToIdle_IT(&huart6, GPSbuffer1, sizeof(GPSbuffer1));



		}
		hal_st1 = HAL_UARTEx_ReceiveToIdle_IT(&huart1, GPSbuffer, sizeof(GPSbuffer));

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
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

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 19200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  huart3.Init.BaudRate = 19200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  huart6.Init.BaudRate = 9600;
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

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MS_CS_Pin|LED_Blue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, User_LED_Pin|SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_Green_Pin|LED_Red_Pin|LED_Orange_Pin|LED_Yellow_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MS_CS_Pin LED_Blue_Pin */
  GPIO_InitStruct.Pin = MS_CS_Pin|LED_Blue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : User_LED_Pin SPI_CS_Pin */
  GPIO_InitStruct.Pin = User_LED_Pin|SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Green_Pin LED_Red_Pin LED_Orange_Pin LED_Yellow_Pin */
  GPIO_InitStruct.Pin = LED_Green_Pin|LED_Red_Pin|LED_Orange_Pin|LED_Yellow_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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

