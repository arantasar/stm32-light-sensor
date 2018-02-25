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
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#include "math.h"
#include "stdlib.h"



/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

// OD CZUJNIKA

#define TSL2561_ADDR 					(0x39 << 1)
#define TSL2561_CMD 					0x80
#define TSL2561_CMD_CLEAR 		0xC0
#define	TSL2561_REG_CONTROL 	0x00
#define	TSL2561_REG_TIMING 		0x01
#define	TSL2561_REG_THRESH_L 	0x02
#define	TSL2561_REG_THRESH_H 	0x04
#define	TSL2561_REG_INTCTL 		0x06
#define	TSL2561_REG_ID 				0x0A
#define	TSL2561_REG_DATA_0 		0x0C
#define	TSL2561_REG_DATA_1 		0x0E
#define RESULTS_SIZE					10

uint8_t wlacz_czujnik[] = {TSL2561_CMD | TSL2561_REG_CONTROL, 0x03};
uint8_t wylacz_czujnik[] = {TSL2561_CMD | TSL2561_REG_CONTROL, 0x00};

uint8_t ch0 = TSL2561_CMD | TSL2561_REG_DATA_0;
uint8_t ch1 = TSL2561_CMD | TSL2561_REG_DATA_1;

uint8_t ch0Raw[2], ch1Raw[2], resultsCounter;
uint16_t channel0, channel1, lux, speed, currentLux, currentIR,
	luxResults[RESULTS_SIZE], infraredResults[RESULTS_SIZE];

volatile uint8_t resultsWriteIndex, resultsReadIndex;
uint32_t tickstart;

float luxMean, infraredMean, luxSD, infraredSD; 

// OD USARTA

#define RX_BUFFER_SIZE 				1024
#define TX_BUFFER_SIZE 				1024
#define QUEUE_SIZE						1024
#define COMMAND_BUFFER_SIZE 	7
#define PARAM_SIZE 						4
#define LUX_MESSAGE						"LUX\t Current: XXXXXX, Mean: XXXXXX.XX, SD: XXXXXX.XX\n"
#define IR_MESSAGE						" IR\t Current: XXXXXX, Mean: XXXXXX.XX, SD: XXXXXX.XX\n"

uint8_t rxBuffer[RX_BUFFER_SIZE], txBuffer[TX_BUFFER_SIZE];
uint16_t commandQueue[QUEUE_SIZE], paramQueue[QUEUE_SIZE];
uint8_t commandBuffer[COMMAND_BUFFER_SIZE];
uint8_t param[PARAM_SIZE];
uint8_t command, res, isOn, isLuxTransmitOn, isIRTransmitOn;

volatile uint16_t counter, writeIndex, readIndex, commandCounter, i, j,
	commandWriteIndex, commandReadIndex, txWriteIndex, stopTx, txReadIndex, txCounter;

char temp;





/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

// OD CZUJNIKA

void powerUpSensor(void);
void powerDownSensor(void);
void changeInterval(uint16_t interval);
unsigned int getLux(void);
extern unsigned int CalculateLux(unsigned int iGain, unsigned int tInt, unsigned int ch0,
unsigned int ch1, int iType);
void getMean(uint16_t start, uint16_t stop, uint16_t n);
void getSD(uint16_t start, uint16_t stop, uint16_t n);

// OD USARTA

char RxBufferGetChar(void);
void TxBufferPutChar(uint8_t character);
void grabCommand(void);
void processCommand(void);
uint8_t checkCommand(void);
void addToCommandQueue(void);
uint16_t digit_to_int(char d);
void executeCommand(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void addToTx(uint8_t word[]);




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
  MX_I2C1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
	tickstart = HAL_GetTick();
	speed = 2000;
	
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_TC);
	HAL_UART_Receive_IT(&huart2, &rxBuffer[writeIndex], 1);
	

	
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
		if(isOn == 1) {
			
			if(HAL_GetTick() - tickstart >= speed) {
			
			luxResults[resultsWriteIndex] = getLux();
			currentLux = luxResults[resultsWriteIndex];
			infraredResults[resultsWriteIndex] = channel1;
			currentIR = infraredResults[resultsWriteIndex];
			
			if (resultsCounter < RESULTS_SIZE) {
				getMean(0, resultsWriteIndex, resultsWriteIndex);
				getSD(0, resultsWriteIndex, resultsWriteIndex);
			} else {
				getMean(0, RESULTS_SIZE, RESULTS_SIZE);
				getSD(0, RESULTS_SIZE, RESULTS_SIZE);
			}
			
			resultsWriteIndex++;
			resultsCounter++;
			if (resultsWriteIndex > RESULTS_SIZE) resultsWriteIndex = 0;
			
			// LUX NA TERMINAL 
			
			if(isLuxTransmitOn) {
				char temp[strlen(LUX_MESSAGE)];
				sprintf(temp, "LUX Current: %7i, Mean: %9.2f, SD: %9.2f\n", currentLux, luxMean, luxSD);
				addToTx((uint8_t*)temp);
			}
			
			// IR NA TERMINAL
			
			if(isIRTransmitOn) {
				char temp[strlen(IR_MESSAGE)];
				sprintf(temp, " IR Current: %7i, Mean: %9.2f, SD: %9.2f\n", currentIR, infraredMean, infraredSD);
				addToTx((uint8_t*)temp);
			}
			
			
			
					
			tickstart = HAL_GetTick();
			
			}
		
		}
		
		
		
		if(counter > 0) {
			
			temp = RxBufferGetChar();
			
			if(temp == 'Y') {
				grabCommand();
				processCommand();
				if(checkCommand()) addToCommandQueue();
			}
		}
			
		if (commandCounter > 0) executeCommand();
		
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Pinout Configuration
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

// OD CZUJNIKA

void powerUpSensor(void) {
	HAL_I2C_Master_Transmit(&hi2c1, TSL2561_ADDR, wlacz_czujnik, 2, 100);
}

void powerDownSensor(void) {
	HAL_I2C_Master_Transmit(&hi2c1, TSL2561_ADDR, wylacz_czujnik, 2, 100);
}

unsigned int getLux(void) {
	HAL_I2C_Master_Transmit(&hi2c1, TSL2561_ADDR, &ch0, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, TSL2561_ADDR, ch0Raw, 2, 100);
	
	channel0 = ch0Raw[1] * 256 + ch0Raw[0];
	
	HAL_I2C_Master_Transmit(&hi2c1, TSL2561_ADDR, &ch1, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, TSL2561_ADDR, ch1Raw, 2, 100);
	
	channel1 = ch1Raw[1] * 256 + ch1Raw[0];

	lux = CalculateLux(0, 2, channel0, channel1, 0);
	return lux;
}

void changeInterval(uint16_t interval) {
	speed = interval;
}

void getMean(uint16_t start, uint16_t stop, uint16_t n) {
	uint16_t i;
	float luxResult = 0;
	float infraredResult = 0;
	
	for(i = start; i < stop; i++) {
		luxResult += luxResults[i];
		infraredResult += infraredResults[i];
	}
	
	luxMean = luxResult / n;
	infraredMean = infraredResult / n;
}

void getSD(uint16_t start, uint16_t stop, uint16_t n) {
	
	uint16_t i;
	float luxResult = 0;
	float infraredResult = 0;
	
	for(i = start; i < stop; i++) {
		luxResult += pow((luxResults[i] - luxMean),2);
		infraredResult += pow((infraredResults[i] - infraredMean),2);
	}
	
	luxSD = sqrt(luxResult / n);
	infraredSD = sqrt(infraredResult / n);
}

// OD USARTA

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	
	writeIndex++;
	counter++;
	if(writeIndex >= RX_BUFFER_SIZE) writeIndex = 0;
	
	HAL_UART_Receive_IT(&huart2, &rxBuffer[writeIndex],1);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	
	

	
	
	if(txCounter > 0) {
		HAL_UART_Transmit_IT(&huart2, &txBuffer[txReadIndex], 1);		
		
		txReadIndex++;
		if(txReadIndex >= TX_BUFFER_SIZE) txReadIndex = 0;
		
		__disable_irq;
		txCounter--;
		__enable_irq;
		
	} else if (txCounter == 0) {
		stopTx = 0;
	}
}



char RxBufferGetChar(void) {
	
	char tmp;
	
	tmp = rxBuffer[readIndex];
	
	readIndex++;
	
	if (readIndex >= RX_BUFFER_SIZE) readIndex = 0;
	
	__disable_irq;
	counter--;
	__enable_irq;
	
		
	return tmp;
}

void TxBufferPutChar(uint8_t character) {
	
	txBuffer[txWriteIndex] = character;
	txWriteIndex++;
	
	txCounter++;
	
	if(txWriteIndex >= TX_BUFFER_SIZE) txWriteIndex = 0;
} 

void grabCommand(void) {
	for(i = 0; i < COMMAND_BUFFER_SIZE; i++) {
		j = (readIndex + RX_BUFFER_SIZE - COMMAND_BUFFER_SIZE + i) % RX_BUFFER_SIZE;
		commandBuffer[i] = rxBuffer[j];
	} 
}

void processCommand(void) {
	if(commandBuffer[1] == 'X') command = commandBuffer[0];
	for(i = 0; i < PARAM_SIZE; i++) param[i] = commandBuffer[i+2];
}

uint16_t digit_to_int(char d)
{
 char str[2];

 str[0] = d;
 str[1] = '\0';
 return (uint16_t) strtol(str, NULL, 10);
}

uint8_t checkCommand(void) {
	
	res = 0;
		
	if(digit_to_int(command) > 0 && digit_to_int(command) < 6) {
		for(i = 0; i < PARAM_SIZE; i++) {
			if(param[i] >= 48 && param[i] <= 57) res++;		
		}		
	}
	
	return res == 4 ? 1 : 0;
}

void addToCommandQueue(void) {
	
	commandQueue[commandWriteIndex] = digit_to_int(command);
	paramQueue[commandWriteIndex] = digit_to_int(param[0]) * 1000 + digit_to_int(param[1]) * 100 
		+ digit_to_int(param[2]) * 10 + digit_to_int(param[3]);
	
	commandWriteIndex++;
	
	if(commandWriteIndex >= QUEUE_SIZE) commandWriteIndex = 0;
	commandCounter++;
	
}

void addToTx(uint8_t word[]) {
	
	for (i = 0; i < strlen((char*)word); i++) {
		TxBufferPutChar(word[i]);
	}
	
	if(txCounter > 0 && stopTx == 0) {
			
			HAL_UART_Transmit_IT(&huart2, &txBuffer[txReadIndex], 1);
			stopTx = 1;
			txReadIndex++;
			if(txReadIndex >= TX_BUFFER_SIZE) txReadIndex = 0;
			txCounter--;
		}
		
}

void executeCommand(void) {
	
	uint8_t cmd = commandQueue[commandReadIndex];
	
	switch(cmd) {
		case 1: // TURN ON / OFF
			if(isOn == 0) { isOn = 1; 	powerUpSensor(); addToTx((uint8_t*)"Czujnik wlaczony\n"); }
			else if (isOn == 1) { isOn = 0; powerDownSensor(); addToTx((uint8_t*)"Czujnik wylaczony\n");} 
			break;
		case 2: // TURN ON / OFF - LUX
			if(isLuxTransmitOn == 0) { isLuxTransmitOn = 1; }
			else if (isLuxTransmitOn == 1) { isLuxTransmitOn = 0; } 
			break;
		case 3: // TURN ON / OFF - IR
			if(isIRTransmitOn == 0) { isIRTransmitOn = 1; }
			else if (isIRTransmitOn == 1) { isIRTransmitOn = 0; } 
			break;
		case 4: // CHANGE SPEED
			if(paramQueue[commandReadIndex] > 0) { speed = paramQueue[commandReadIndex]; }
			break;
		case 5: // SENSOR REGISTER ACCESS
			if(paramQueue[commandReadIndex] >= 0 && paramQueue[commandReadIndex] <= 15) {
				uint8_t reg, command = TSL2561_CMD | paramQueue[commandReadIndex], temp[4];
				HAL_I2C_Master_Transmit(&hi2c1, TSL2561_ADDR, &command, 1, 100);
				HAL_I2C_Master_Receive(&hi2c1, TSL2561_ADDR, &reg, 1, 100);
				sprintf((char*)temp, "0x%02x", reg);
				addToTx(temp);		
			}
			break;
	}
	
	commandReadIndex++;
	if(commandReadIndex >= QUEUE_SIZE) commandReadIndex = 0;
	commandCounter--;
	
}

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
