/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "pin_map.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define NAME_SIZE 14
#define BKRAM_MESSAGE_STAY_IN_BOOT (uint16_t)0x1989
#define DEFAULT_DEVICE_CAN_ID                           0x78E
#define DEFAULT_TX_CAN_ID                           	0x78F
#define DEFAULT_SEC_KEY 0xfe123bca
#define DEFAULT_TIME_BEFORE_START 5000

typedef enum {
    WAIT_HOST = 0,
    IDLE      = 1,
    PAGE_PROG = 2,
    MAIN_SPACE_INCORRECT = 3,
    PAGE_PROG_END = 4,
    STATE_START_MAIN_PROGRAM = 5,
}device_state_t;

typedef enum {
    HOST_CMD_INIT               = 0x01,
    HOST_CMD_START_PAGE_LOAD    = 0x02,
    HOST_CMD_START_MAIN_APPL	= 0x03,
    HOST_CMD_GET_GUID 		    = 0x04,/*!< crc id - u32 size*/
    HOST_CMD_END_PAGE_LOAD      = 0x05,
    HOST_CMD_GET_NAME           = 0x06,
    HOST_CMD_GET_APPL_INFO      = 0x07,/*!< size and crc*/
    HOST_CMD_GET_CAN_INFO       = 0x08,/*!< addres and baud rate*/
    HOST_CMD_GET_COUNT_AND_TIME_INFO       = 0x09,/*!< counter and unix time*/
    HOST_CMD_SET_NAME1           = 0x0a,/*!< set first part of name*/
    HOST_CMD_SET_NAME2           = 0x0b,/*!< set second part of name*/
    HOST_CMD_SET_CAN_INFO        = 0x0c,/*!< set size and crc*/
    HOST_CMD_SET_TIME            = 0x0d,/*!< set unix time*/
    HOST_CMD_SWITCH_TO_BOOT      = 0x0e,/*!< controller will switch and stay into boot mode*/
}host_commands_t;

typedef enum {
    DEVICE_RESP_OK                          = 0x01,
    DEVICE_RESP_ERROR                      	= 0x02,
    DEVICE_RESP_BOOTING						= 0x03,
    DEVICE_RESP_PAGE_FLASH_CRC_ERROR         = 0x05,
    DEVICE_RESP_ACKNOWLEDGE      = 0x06,/*!< acknowledge command*/
    DEVICE_RESP_PAGE_END_ACKNOWLEDGE      = 0x07,/*!< acknowledge command*/
}device_commands_t;

typedef struct MCU_PACK{
    uint32_t size_of_table;
    uint32_t main_program_size;
    uint32_t main_program_crc;
    uint32_t flash_write_counter;
    int32_t last_flashed_unix_time;
    uint32_t reserv;
    uint16_t boot_loader_config;
    uint16_t flash_security;
    uint32_t security_key;
    uint32_t time_before_start;
    uint16_t can_address;
    uint16_t can_baud_rate;
    uint8_t  name[NAME_SIZE];
    uint16_t reserv_1;
    uint32_t crc;
}stored_struct_t;

typedef union {
    stored_struct_t stored_struct;
    uint32_t bytes[FLASH_PAGE_SIZE/4];
}stored_data_t;
typedef enum {
    S0_10KB =0,  //S0 = 10 kBaud
    S1_20KB,//S1 = 20 kBaud
    S2_50KB,//S2 = 50 kBaud
    S3_100KB,//S3 = 100 kBaud
    S4_125KB,//S4 = 125 kBaud
    S5_250KB,//S5 = 250 kBaud
    S6_500KB,//S6 = 500 kBaud
    S7_800KB,//S7 = 800 kBaud
    S8_1000KB//S8 = 1 MBaud
}can_baudrate_t;
union CANData
{
    uint8_t  b[8];
    uint16_t d[4];
    uint32_t w[2];
};
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define MCU_PACK __attribute__((packed))
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern CAN_HandleTypeDef hcan;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
