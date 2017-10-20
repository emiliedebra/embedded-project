/**
  ******************************************************************************
  * @file    Audio/Audio_playback_and_record/Src/main.c 
  * @author  ES Team 5
  * @version V1.3.5
  * @date    October-2017
  * @brief   Main program body.
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define TXBUFFERSIZE 1
#define RXBUFFERSIZE 1

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef UartHandle;
__IO ITStatus UartReady = RESET;

TIM_HandleTypeDef TimHandle;
TIM_HandleTypeDef hTimLed;
uint32_t uwPrescalerValue = 0;

uint8_t aTxBuffer[TXBUFFERSIZE];
uint8_t aRxBuffer[RXBUFFERSIZE];

/* Counter for User button presses. Defined as external in waveplayer.c file */
__IO uint32_t PressCount = 0;
__IO uint32_t PbPressCheck = 0;

/* Wave Player Pause/Resume Status. Defined as external in waveplayer.c file */
__IO uint32_t PauseResumeStatus = IDLE_STATUS;
__IO uint32_t CmdIndex = CMD_PLAY;

extern uint32_t AudioPlayStart;

/* Re-play Wave file status on/off.
   Defined as external in waveplayer.c file */
__IO uint32_t RepeatState = REPEAT_ON;

/* Capture Compare Register Value.
   Defined as external in stm32f4xx_it.c file */
__IO uint16_t CCR1Val = 16826;              
                                            
extern __IO uint32_t LEDsState;

/* Beat Tracking Variables */
uint8_t beat[1] = {0};
uint8_t buttons[2] = {0x00, 0x00}; // TODO: Make this an array of 8 char's and then check all index's of each char on each beat play - add sounds - play
// char * buttonChars[8] = {};
uint8_t beatFlag = 0;

/* USB Variables */
FATFS USBDISKFatFs;          /* File system object for USB disk logical drive */
char USBDISKPath[4];         /* USB Host logical drive path */
USBH_HandleTypeDef hUSBHost; /* USB Host handle */
static uint8_t  USBH_USR_ApplicationState = USBH_USR_FS_INIT;

MSC_ApplicationTypeDef AppliState = APPLICATION_IDLE;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void USBH_UserProcess(USBH_HandleTypeDef *pHost, uint8_t vId);
static void MSC_Application(void);
void playBeat();

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32F4xx HAL library initialization:
     - Configure the Flash prefetch, instruction and Data caches
     - Configure the Systick to generate an interrupt each 1 msec
     - Set NVIC Group Priority to 4
     - Global MSP (MCU Support Package) initialization
  */
  HAL_Init();
  
  /* Configure LED3, LED4, LED5 and LED6 */
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);
  BSP_LED_Init(LED5);
  BSP_LED_Init(LED6);

  /* Configure the system clock to 168 MHz */
  SystemClock_Config();

  /* Timer Config */
  uwPrescalerValue = (uint32_t) ((SystemCoreClock /2) / 10000) - 1;
  TimHandle.Instance = TIM3;
  TimHandle.Init.Period = 16000 - 1;
  TimHandle.Init.Prescaler = uwPrescalerValue;
  TimHandle.Init.ClockDivision = 0;
  TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  {
	  /* Initialization Error */
	  Error_Handler();
  }
  /* Start the TIM Base generation in interrupt mode*/
  /* Start Channel1 */
  if(HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
  {
	  /* Starting Error */
	  Error_Handler();
  	  }

  /* Initialize the Repeat state */
  RepeatState = REPEAT_ON;

  /* Configure USER Button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
  
  /* UART Config */
  UartHandle.Instance = USARTx;
  UartHandle.Init.BaudRate = 9600;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits = UART_STOPBITS_1;
  UartHandle.Init.Parity = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode = UART_MODE_TX_RX;
  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

  if(HAL_UART_Init(&UartHandle) != HAL_OK) {
	  /* Starting Error */
	  Error_Handler();
  }

  /* Turn ON LED4: start of application */
  BSP_LED_On(LED4);

  /*##-1- Link the USB Host disk I/O driver */
  if(FATFS_LinkDriver(&USBH_Driver, USBDISKPath) == 0)
  {
    /*##-2- Init Host Library */
    USBH_Init(&hUSBHost, USBH_UserProcess, 0);

    /*##-3- Add Supported Class */
    USBH_RegisterClass(&hUSBHost, USBH_MSC_CLASS);

    /*##-4- Start Host Process */
    USBH_Start(&hUSBHost);

    /* Run Application (Blocking mode)*/
    while (1)
    {
    	// TODO: receive and set volume
    	// TODO: receive and set tempo

    	if (beatFlag == 1){
    		//compute the beat
    		//send beat to F0
			if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t *)beat, 1)!= HAL_OK){
				Error_Handler();
			}
			while(UartReady != SET){}
			UartReady = RESET;

			//receive button array
			if(HAL_UART_Receive_IT(&UartHandle, (uint8_t *)buttons, 2)!= HAL_OK){
				Error_Handler();
			}
			while(UartReady != SET){}
			UartReady = RESET;

			//play the beat
			if(buttons[beat[0]] == 1){ // TODO: Check if buttons[beat[0].charAt(i)] kinda vibes
				playBeat();
			}

			// TODO: Implement for 8x8
			//move to the next beat
			if (beat[0] == 1){ // if beat[0] == 8
				beat[0] = 0;
			}
			else{
				beat[0]++;
			}

			//finished computing beat flag
    		beatFlag = 0;
    	}

    	switch(AppliState)
    	{
    		case APPLICATION_START:
    			MSC_Application();
    			break;
    		case APPLICATION_IDLE:
    		default:
    			break;
    	}
    	/* USBH_Background Process */
    	USBH_Process(&hUSBHost);
    }
  }
}

void playBeat() {
	CmdIndex = CMD_PLAY;
	BSP_LED_On(LED5);
	WavePlayerStart();
	BSP_LED_Off(LED5);
}


/* Beat Timer Interrupt */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *TimHandle)
{
	BSP_LED_Toggle(LED6);
	beatFlag = 1;
}

/* Transfer Interrupt */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle) {
	UartReady = SET;
}

/* Receiver Interrupt */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {
	UartReady = SET;
}

/**
  * @brief  User Process
  * @param  phost: Host Handle
  * @param  id: Host Library user message ID
  * @retval None
  */
static void USBH_UserProcess (USBH_HandleTypeDef *pHost, uint8_t vId)
{  
  switch (vId)
  { 
  case HOST_USER_SELECT_CONFIGURATION:
    break;
    
  case HOST_USER_DISCONNECTION:
    WavePlayer_CallBack();
    AppliState = APPLICATION_IDLE;
    f_mount(NULL, (TCHAR const*)"", 0);          
    break;
    
  case HOST_USER_CLASS_ACTIVE:
    AppliState = APPLICATION_START;
    break;
    
  case HOST_USER_CONNECTION:
    break;
    
  default:
    break; 
  }
}

/**
  * @brief  Main routine for Mass storage application
  * @param  None
  * @retval None
  */
static void MSC_Application(void)
{
  switch (USBH_USR_ApplicationState)
  {
  case USBH_USR_AUDIO:
    /* Set user initialization flag */
    USBH_USR_ApplicationState = USBH_USR_FS_INIT;
    break;
    
  case USBH_USR_FS_INIT:
    /* Initializes the File System */
    if (f_mount(&USBDISKFatFs, (TCHAR const*)USBDISKPath, 0 ) != FR_OK ) 
    {
      /* FatFs initialisation fails */
      Error_Handler();
    }
    
    /* Go to menu */
    USBH_USR_ApplicationState = USBH_USR_AUDIO;
    break;
    
  default:
    break;
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
  if (HAL_GetREVID() == 0x1001)
  {
    /* Enable the Flash prefetch */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  }  
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* Turn LED3 on */
  BSP_LED_On(LED3);
  while(1)
  {
  }
}

 /**
  * @brief  EXTI line detection callbacks - used for playing on button press.
  * @param  GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_0) 
  {
	  CmdIndex = CMD_PLAY;
	  BSP_LED_On(LED4);
	  WavePlayerStart(); // TODO: Check that this isn't calling a change buffer method
	  HAL_Delay(10);
	  BSP_LED_Off(LED4);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle) {
	BSP_LED_On(LED3);
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
