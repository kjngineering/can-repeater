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
#include "string.h"
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
CAN_HandleTypeDef hcan;
static CRC_HandleTypeDef hcrc;
/* USER CODE BEGIN PV */
#ifndef DEBUG
#define DEBUG 0
#endif
#define HARD_LEN_KEY   0xE2E4E6E8
#define LENGTH_CODE __attribute__ ((section (".length_space")))
static volatile unsigned long const sofi_debug_len  LENGTH_CODE = HARD_LEN_KEY;
extern uint32_t _store_space_address;
extern uint32_t _store_space_size;
extern uint32_t _main_program_pages;
static uint32_t crc_id = 0;

#define STORE_SPACE_ADDRESS_LD ((uint32_t)&_store_space_address)
#define STORE_SPACE_SIZE_LD ((uint32_t)&_store_space_size)
#define APP_PAGES ((uint32_t)&_main_program_pages)

static char default_name[NAME_SIZE] = "default_name";
static CAN_TxHeaderTypeDef canTxMessage;
static CAN_RxHeaderTypeDef canRxMessage;
static FLASH_EraseInitTypeDef eraseInitStruct;
static CAN_FilterTypeDef sf;
static uint32_t TxMailbox;
static union CANData TxData;
static union CANData RxData;
static stored_data_t stored_data;
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static uint32_t calc_id_crc(void);
static void MX_CRC_Init(void);
static int bkram_init(void);
static void can_init(void);
static void transmit_response(uint8_t * response,uint8_t len);
static void transmit_data(uint32_t id,uint8_t * response,uint8_t len);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* IWDG init function */
static IWDG_HandleTypeDef hiwdg;
void MX_IWDG_Init(void){    // IWDG_PERIOD = Reload*Prescaler/32000 sec     (8.2 sec)
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    hiwdg.Init.Reload = 4095;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK){

    }
}
/**
 * @brief read stored data from flash
 */
static void read_stored_flash(stored_data_t *stored_data){
    HAL_FLASH_Unlock();
    memcpy(stored_data->bytes,(void*)STORE_SPACE_ADDRESS_LD,FLASH_PAGE_SIZE);
    HAL_FLASH_Lock();
}
/**
 * @brief save_stored_struct
 * @param stored_data
 * @return none zero value if an error occured
 */
static int save_stored_struct(stored_data_t *stored_data){
    HAL_FLASH_Unlock();
    uint32_t PageError = 0;
    eraseInitStruct.TypeErase = TYPEERASE_PAGES;
    eraseInitStruct.PageAddress = STORE_SPACE_ADDRESS_LD;
    eraseInitStruct.NbPages = 1;
    HAL_FLASHEx_Erase(&eraseInitStruct, &PageError);
    for (uint32_t i = 0; i < sizeof(stored_struct_t); i+=4){
        HAL_FLASH_Program(TYPEPROGRAM_WORD, STORE_SPACE_ADDRESS_LD + i, stored_data->bytes[i/4]);
    }
    HAL_FLASH_Lock();
    return 0;
}/**
 * @brief check_stored_struct
 * @param stored_data
 * @return 1 if stored 0- if crc mismatch <0 if error occured
 */
static int check_stored_struct(stored_data_t *stored_data){
    int res = 0;
    if (stored_data->stored_struct.size_of_table<FLASH_PAGE_SIZE){
        if(HAL_CRC_Calculate(&hcrc, (void*)stored_data->bytes, (stored_data->stored_struct.size_of_table) / 4)==0){
            res = 1;
        }
    }
    if (res !=1){
        //not compared
        memcpy(stored_data->stored_struct.name,default_name,NAME_SIZE);
        stored_data->stored_struct.reserv = 0;
        stored_data->stored_struct.can_address = DEFAULT_DEVICE_CAN_ID;
        stored_data->stored_struct.security_key = DEFAULT_SEC_KEY;
        stored_data->stored_struct.can_baud_rate = S6_500KB;
        stored_data->stored_struct.size_of_table = sizeof(stored_struct_t);
        stored_data->stored_struct.flash_security = 0;
        stored_data->stored_struct.main_program_crc = 0;
        stored_data->stored_struct.main_program_size = 0;
        stored_data->stored_struct.time_before_start = DEFAULT_TIME_BEFORE_START;
        stored_data->stored_struct.boot_loader_config = 0;
        stored_data->stored_struct.flash_write_counter = 0;
        stored_data->stored_struct.last_flashed_unix_time = 0;
        stored_data->stored_struct.crc = HAL_CRC_Calculate(&hcrc, (void*)stored_data->bytes, (sizeof(stored_struct_t)-4)/4);
        res = save_stored_struct(stored_data);
    }
    return res;
}
static uint8_t command_receive;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
    HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&canRxMessage,RxData.b);
    // Skip messages not intended for our device
    if ((canRxMessage.StdId == stored_data.stored_struct.can_address) ||
            (canRxMessage.StdId == DEFAULT_DEVICE_CAN_ID)) {
        if (canRxMessage.DLC==8){
        }else{
            command_receive = RxData.b[0];
        }
        HAL_GPIO_TogglePin(LED_0_GPIO_PORT, LED_0_PIN);
    }
    return;
}

static void transmit_response(uint8_t * response,uint8_t len){
    len = len>8?1:len;
    canTxMessage.StdId = DEFAULT_TX_CAN_ID;
    canTxMessage.IDE = CAN_ID_STD;
    canTxMessage.RTR = CAN_RTR_DATA;
    canTxMessage.DLC = len;
    canTxMessage.TransmitGlobalTime = DISABLE;
    memcpy(TxData.b,response,len);
    if (HAL_CAN_AddTxMessage(&hcan, &canTxMessage, TxData.b, &TxMailbox) != HAL_OK) {

    }
}

static void transmit_data(uint32_t id,uint8_t * response,uint8_t len){
    len = len>8?1:len;


   //write some ID extension in here -- if ID > 0x7ff then switch to extended. If ID > extended range 2^29 exemption.

    canTxMessage.StdId = id;
    canTxMessage.IDE = CAN_ID_STD;
    canTxMessage.RTR = CAN_RTR_DATA;
    canTxMessage.DLC = len;
    canTxMessage.TransmitGlobalTime = DISABLE;
    memcpy(TxData.b,response,len);
    if (HAL_CAN_AddTxMessage(&hcan, &canTxMessage, TxData.b, &TxMailbox) != HAL_OK) {

    }
}

static uint32_t calc_id_crc(){
    uint8_t temp_buff[12] = {0};
    memcpy(temp_buff,(uint8_t*)UID_BASE,12);
    uint32_t crc_mac = HAL_CRC_Calculate(&hcrc,(void*)temp_buff, 12/4);
    return crc_mac;
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
	//SystemInit();
  SCB->VTOR = 0x08002800U;
  HAL_Init();
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */
  /* Configure the system clock */
  SystemClock_Config();
  /* USER CODE BEGIN SysInit */
  MX_IWDG_Init();
  /* USER CODE END SysInit */
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CRC_Init();
  crc_id = calc_id_crc();
  read_stored_flash(&stored_data);
  check_stored_struct(&stored_data);
  bkram_init();
  can_init();
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
    /* USER CODE END WHILE */
#if DEBUG
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
#endif
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
      HAL_Delay(2000);
      uint8_t temp[4] = {0xDE,0xAD,0xBE,0xEF};
      transmit_data(0x123,temp, 4);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	  HAL_Delay(1000);

      if(command_receive==HOST_CMD_SWITCH_TO_BOOT){
        command_receive = 0;
        uint8_t temp[2] = {HOST_CMD_START_PAGE_LOAD,DEVICE_RESP_ACKNOWLEDGE};
        transmit_response(temp, 2);
        BKP->DR1 = BKRAM_MESSAGE_STAY_IN_BOOT;
        NVIC_SystemReset();
      }


      if (command_receive==HOST_CMD_GET_GUID){
        command_receive = 0;
        uint8_t temp[8];
        temp[0] = HOST_CMD_GET_GUID;
        temp[1] = DEVICE_TYPE_F103;
        temp[2] = DEVICE_STATUS_APP;
        temp[3] = APP_PAGES;
		memcpy(&temp[4],&crc_id,4);
        transmit_response((uint8_t*)temp, 8);
      }





	  __HAL_IWDG_RELOAD_COUNTER(&hiwdg);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /*Configure GPIO pin : PC13 */
#if DEBUG
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
#endif

}
static int bkram_init(){
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_BKP_CLK_ENABLE();
    return 0;
}
/* USER CODE BEGIN 4 */
/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(){
    hcrc.Instance = CRC;
    if (HAL_CRC_Init(&hcrc) != HAL_OK){
        Error_Handler();
    }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void can_init(void){
    hcan.Instance = CAN1;
    /*input freq 24 mhz*/
/*    10 kBaud Sample Point 75.0 % : 24MHZ / 120 = 200kHz -> TQ 20   SJW + TS1 15 TS2 5
    20 kBaud Sample Point 75.0 % : 24MHZ /  60 = 400kHz -> TQ 20   SJW + TS1 15 TS2 5
    50 kBaud Sample Point 75.0 % : 24MHZ /  24 = 1MHz   -> TQ 20   SJW + TS1 15 TS2 5
   100 kBaud Sample Point 80.0 % : 24MHZ /  24 = 1MHz   -> TQ 10   SJW + TS1  8 TS2 2
   125 kBaud Sample Point 87.5 % : 24MHZ /  12 = 2MHz   -> TQ 16   SJW + TS1 14 TS2 2
   250 kBaud Sample Point 87.5 % : 24MHZ /   6 = 4MHz   -> TQ 16   SJW + TS1 14 TS2 2
   500 kBaud Sample Point 87.5 % : 24MHZ /   6 = 4MHz   -> TQ  8   SJW + TS1  7 TS2 1
   800 kBaud Sample Point 86.7 % : 24MHZ /   2 = 12MHz  -> TQ 15   SJW + TS1 13 TS2 2
  1000 kBaud Sample Point 88.9 % : 24MHZ /   2 = 12MHz  -> TQ 18   SJW + TS1 8 TS2 3
*/
    switch(stored_data.stored_struct.can_baud_rate) {
    case S0_10KB:
        hcan.Init.Prescaler = 120;
        hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
        hcan.Init.TimeSeg1 = CAN_BS1_14TQ;
        hcan.Init.TimeSeg2 = CAN_BS2_5TQ;
        break;
    case S1_20KB:
        hcan.Init.Prescaler = 60;
        hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
        hcan.Init.TimeSeg1 = CAN_BS1_14TQ;
        hcan.Init.TimeSeg2 = CAN_BS2_5TQ;
        break;
    case S2_50KB:
        hcan.Init.Prescaler = 24;
        hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
        hcan.Init.TimeSeg1 = CAN_BS1_14TQ;
        hcan.Init.TimeSeg2 = CAN_BS2_5TQ;
        break;
    case S3_100KB:
        hcan.Init.Prescaler = 24;
        hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
        hcan.Init.TimeSeg1 = CAN_BS1_8TQ;
        hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
        break;
    case S4_125KB:
        hcan.Init.Prescaler = 12;
        hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
        hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
        hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
        break;
    case S5_250KB:
        hcan.Init.Prescaler = 6;
        hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
        hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
        hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
        break;
    case S6_500KB:
        hcan.Init.Prescaler = 3;
        hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
        hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
        hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
       break;
    case S7_800KB:
        hcan.Init.Prescaler = 2;
        hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
        hcan.Init.TimeSeg1 = CAN_BS1_12TQ;
        hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
        break;
    case S8_1000KB:
        hcan.Init.Prescaler = 2;
        hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
        hcan.Init.TimeSeg1 = CAN_BS1_8TQ;
        hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
        break;
    default:
        hcan.Init.Prescaler = 6;
        hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
        hcan.Init.TimeSeg1 = CAN_BS1_7TQ;
        hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
        break;
    }

    hcan.Init.Mode = CAN_MODE_NORMAL;
    hcan.Init.TimeTriggeredMode = DISABLE;
    hcan.Init.AutoBusOff = DISABLE;
    hcan.Init.AutoWakeUp = DISABLE;
    hcan.Init.AutoRetransmission = ENABLE;
    hcan.Init.ReceiveFifoLocked = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan) != HAL_OK)  {
        Error_Handler();
    }
    sf.FilterIdHigh = 0x0000;
    sf.FilterIdLow = 0x0000;
    sf.FilterMaskIdHigh = 0x0000;
    sf.FilterMaskIdLow = 0x0000;
    sf.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sf.FilterBank = 0;
    sf.FilterMode = CAN_FILTERMODE_IDMASK;
    sf.FilterScale = CAN_FILTERSCALE_32BIT;
    sf.FilterActivation = ENABLE;
    sf.SlaveStartFilterBank = 14;
    //Apply Filter to hca
    if (HAL_CAN_ConfigFilter(&hcan, &sf) != HAL_OK) {
        Error_Handler();
    }
    //Start CAN Interface
    if (HAL_CAN_Start(&hcan) != HAL_OK) {
        Error_Handler();
    }
    //Attach Interrupts/Callbacks
    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();
    }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
