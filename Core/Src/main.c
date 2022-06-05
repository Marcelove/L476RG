/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
ADC_HandleTypeDef hadc1;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
int taskDelayValue = 500;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//Tamanho das filas de transmissao e recepcao
//A fila de transmissao é maior do que a de recepcao por que a rotina de transmissao (write_string)
//pode enfileirar um numero maior de bytes de forma muito rapida
#define TX_QUEUE_SIZE 128
#define RX_QUEUE_SIZE 5
#define USART_1 1
#define USART_2 2
//Variaveis que irao armazenar os hanldes das filas de tranmissao e recepcao
QueueHandle_t tx_queue_1;
QueueHandle_t rx_queue_1;
QueueHandle_t tx_queue_2;
QueueHandle_t rx_queue_2;
SemaphoreHandle_t uart_1_mutex = NULL;
SemaphoreHandle_t uart_2_mutex = NULL;

void turnBuzzerOn(void){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,GPIO_PIN_SET);               // Ring Buzzer
}

void turnBuzzerOff(void){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,GPIO_PIN_RESET);	            //Remain silent
}


void sendchar(char c, char usart){
	if(usart == USART_1){
		xQueueSend(tx_queue_1, &c, HAL_MAX_DELAY);
		LL_USART_EnableIT_TXE(USART1);
	}
	else if( usart == USART_2){
		xQueueSend(tx_queue_2, &c, HAL_MAX_DELAY);
		LL_USART_EnableIT_TXE(USART2);
	}
}

void sendString(char * str, char usart){
	while(*str != 0){
		sendchar(*str, usart);
		str++;
	}
}

char readchar(char usart){
	uint8_t caracter=0;
	if(usart == USART_1)
		xQueueReceive(rx_queue_1, &caracter, HAL_MAX_DELAY);
	else if(usart == USART_2)
		xQueueReceive(rx_queue_2, &caracter, HAL_MAX_DELAY);
	return caracter;
}

int16_t readVoltage(void){
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	return HAL_ADC_GetValue(&hadc1);
}


void blinkTaskFcn(void *argument)
{
	for(;;)
	{
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		if(taskDelayValue <= 0)
			break;
		else
			vTaskDelay(taskDelayValue);
	}
}

void readButtonState()
{
	for(;;){
		if(!(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin))){
			taskDelayValue -= 25;
			while(!(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)));
			vTaskDelay(500);
		}
	}

}

int readPresence(void)
{
	if(!(HAL_GPIO_ReadPin(B1_button_GPIO_Port, B1_button_Pin))){
		return 1;

	}
	else{
		return 0;
	}
}

void lightControl(void * vParam)
{
	int refletor = 0;

	//TESTAR LUZ
	while(1){
		float luz = (float)readVoltage();
		luz = (luz * 3.3) / 4096;
		//REGRA DE TRES PARA CONVERSAO
		// 3.3 ----- 400
		luz = luz * 124.2;
		//Verificar a cada 10 minutos a luminosidade
		//vTaskDelay(60000);
		vTaskDelay(600);

		if (luz < 70){

			if (refletor == 0){
				sendString("Ambiente escuro. Acendendo refletor...\r\n", USART_2);
				lightLobbyUp();
				refletor = 1;
			}
		}
		else{
			lightLobbyDown();
			refletor = 0;
			//sendString("Ambiente Claro.\r\n", USART_2);
		}
	}
}

void lightLobbyUp()
{
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
}

void lightLobbyDown()
{
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
}

void unlockDoor(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);               //Liberar porta
}

void lockDoor(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);               //fechar porta
}

int verificaFirebase(void)
{
	char buff[8];
	sendString("Teste Serial\r\n", USART_1);

	char respostaFirebase;
	//int respostaNumerica;
	respostaFirebase = readchar(USART_1);
	sendString("--------\r\n", USART_2);
	sendString("Verificando autenticacao...\r\n", USART_2);

	/*
	sprintf_(buff, "%d\r\n", respostaFirebase);
	sendString(buff, USART_2);
	*/


	//respostaNumerica = atoi(respostaFirebase);
	//sprintf_(buff, "%c\r\n", respostaFirebase);
	//sendString(buff, USART_2);

	//respostaNumerica = atoi(respostaFirebase);

	if (respostaFirebase == 'S')
			return 1;
	else
		return 0;
}

void cli(void * vParam)
{
	uint8_t caracter;
	int  autorizado = 0;
	int presenca = 0;

	while(1)
	{
		sendString("--------\r\n", USART_2);
		sendString("----PORTARIA REMOTA----\r\n", USART_2);
		sendString("--------\r\n", USART_2);

		//Enquanto não houver presença
		while(presenca == 0)
		{
			//TESTAR BUZZER
			//turnBuzzerOn();
			//vTaskDelay(500);
			//turnBuzzerOff();
			sendString("--------\r\n", USART_2);
			sendString("Presenca nao detectada.\r\n", USART_2);

			//Verifica presença a cada 5 segundos
			vTaskDelay(5000);
			presenca = readPresence();
		}

		sendString("--------\r\n", USART_2);
		sendString("Presenca detectada. Pressione 'S'\r\n", USART_2);
		sendString("para liberar a porta apos ter efetuado sua autenticacao.\r\n", USART_2);
		//xQueueReceive(rx_queue, &caracter, HAL_MAX_DELAY);
		caracter = readchar(USART_2);

		//solicitou entrada
		if(caracter == 'S'){

			//Verificar autorização na FireBase
			autorizado = verificaFirebase();
			//autorizado = 1;

			//Autorizado
			if (autorizado == 1){

				sendString("--------\r\n", USART_2);
				sendString("Bem vindo!\r\n", USART_2);
				sendString("Porta aberta durante 10 segundos.\r\n", USART_2);
				sendString("....\r\n", USART_2);
				//Abrir porta (simulando pelo LED da porta)
				unlockDoor();
				//Tem 10 segundos para entrar
				vTaskDelay(10000);
				//Fechando a porta (simulando pelo LED da porta)
				lockDoor();
				autorizado = 0;
				sendString("Porta trancada.\r\n", USART_2);
				vTaskDelay(3000);
			}
			else{
			//Não está Autorizado
			turnBuzzerOn();
			sendString("--------\r\n", USART_2);
			sendString("Permissao negada. Falha na autenticacao.\r\n", USART_2);
			//Tocar Sirene por 2 segundos
			vTaskDelay(2000);
			turnBuzzerOff();
			sendString("Tente novamente.\r\n", USART_2);
			vTaskDelay(6000);
			}
		}
		presenca = 0;
		sendString("----------------\r\n", USART_2);
		sendString("----------------\r\n", USART_2);
		sendString("----------------\r\n", USART_2);
		sendString("----------------\r\n", USART_2);
		sendString("----------------\r\n", USART_2);
		sendString("----------------\r\n", USART_2);
		sendString("----------------\r\n", USART_2);
		sendString("----------------\r\n", USART_2);
	}
}

//Rotina de tratamento de interrupcao da USART2
void USART_2_IRQHandler(void)
{
	//Se for interrupcao de transmissao
	if (LL_USART_IsActiveFlag_TXE(USART2)) {
		BaseType_t contextSwitch;
		while (LL_USART_IsActiveFlag_TXE(USART2)) {
			uint8_t byte;
			//Desinfileira um byte para tranmistir
			if (xQueueReceiveFromISR(tx_queue_2, &byte, &contextSwitch) == pdFAIL) {
				//Se a fila de transmissao esta vazia, encerra a transmissao
				LL_USART_DisableIT_TXE(USART2);
				break;
			}
			//Envia o byte retirado da fila de transmisao
			LL_USART_TransmitData8(USART2, byte);
		}
		portYIELD_FROM_ISR(contextSwitch);
	}
	//Se for interrupcao de recepcao
	if (LL_USART_IsActiveFlag_RXNE(USART2)) {
		BaseType_t contextSwitch;
		while (LL_USART_IsActiveFlag_RXNE(USART2)) {
			//Copia o byte do regstrador de recepcao
			uint8_t byte =
			LL_USART_ReceiveData8(USART2);
			//Enfileira o byte recebido na fila de recepcao
			xQueueSendFromISR(rx_queue_2, &byte, &contextSwitch);
			// xQueueSendFromISR can return errQUEUE_FULL
		}
		portYIELD_FROM_ISR(contextSwitch);
	}
}

//Rotina de trtaamento da ISR da UART 1
void USART_1_IRQHandler(void)
{
	//Se for interrupcao de transmissao
	if (LL_USART_IsActiveFlag_TXE(USART1)) {
		BaseType_t contextSwitch;
		while (LL_USART_IsActiveFlag_TXE(USART1)) {
			uint8_t byte;
			//Desinfileira um byte para tranmistir
			if (xQueueReceiveFromISR(tx_queue_1, &byte, &contextSwitch) == pdFAIL) {
				//Se a fila de transmissao esta vazia, encerra a transmissao
				LL_USART_DisableIT_TXE(USART1);
				break;
			}
			//Envia o byte retirado da fila de transmisao
			LL_USART_TransmitData8(USART1, byte);
		}
		portYIELD_FROM_ISR(contextSwitch);
	}
	//Se for interrupcao de recepcao
	if (LL_USART_IsActiveFlag_RXNE(USART1)) {
		BaseType_t contextSwitch;
		while (LL_USART_IsActiveFlag_RXNE(USART1)) {
			//Copia o byte do regstrador de recepcao
			uint8_t byte =
			LL_USART_ReceiveData8(USART1);
			//Enfileira o byte recebido na fila de recepcao
			xQueueSendFromISR(rx_queue_1, &byte, &contextSwitch);
			// xQueueSendFromISR can return errQUEUE_FULL
		}
		portYIELD_FROM_ISR(contextSwitch);
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  uart_1_mutex = xSemaphoreCreateMutex();
  uart_2_mutex = xSemaphoreCreateMutex();
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  xTaskCreate(cli,
			  "cli",
			  2 * configMINIMAL_STACK_SIZE,
			  NULL,
			  1,
			  NULL);

  xTaskCreate(lightControl,
  			  "lightControl",
  			  configMINIMAL_STACK_SIZE,
  			  NULL,
  			  tskIDLE_PRIORITY,
  			  NULL);
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9|LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */
  //Inicializa as filas de transmissao e recepcao
  tx_queue_1 = xQueueCreate(TX_QUEUE_SIZE, sizeof(uint8_t));
  rx_queue_1 = xQueueCreate(1, sizeof(char));
  //Habilita a interrupcao de recepcao pela USART2
  LL_USART_EnableIT_RXNE(USART1);
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

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2|LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART2 interrupt Init */
  NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(USART2_IRQn);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART2);
  LL_USART_Enable(USART2);
  /* USER CODE BEGIN USART2_Init 2 */
  //Inicializa as filas de transmissao e recepcao
  tx_queue_2 = xQueueCreate(TX_QUEUE_SIZE, sizeof(uint8_t));
  rx_queue_2 = xQueueCreate(RX_QUEUE_SIZE, sizeof(uint8_t));
  //Habilita a interrupcao de recepcao pela USART2
  LL_USART_EnableIT_RXNE(USART2);
  LL_USART_EnableIT_RXNE(USART2);
  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA8 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_button_Pin */
  GPIO_InitStruct.Pin = B1_button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_button_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
