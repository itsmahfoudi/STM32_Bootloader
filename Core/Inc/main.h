/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_crc.h>
#include <stm32f4xx_hal_def.h>
#include <stm32f4xx_hal_gpio.h>
#include <stm32f4xx_hal_uart.h>
#include <string.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define FLASH_SECTOR2_BASE_ADDRESS 0x08008000U
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
//++++++Bootloader supported commands++++++++//
// This command is used to get the Bootloader version from the mcu
#define BL_GET_VER 0x51
// This command is used to get the supported commands by the Bootloader
#define BL_GET_HELP 0x52
// This command is used to get the MCU chip identification number
#define BL_GET_CID 0x53
// This command is used to get the FLASH Read protection level.
#define BL_GET_RDP_STATUS 0x54
// This command is used to jump Bootloader to specific address.
#define BL_GO_TO_ADDR 0x55
// This command is used to erase sectors of the user flash.
#define BL_FLASH_ERASE 0x56
// This command is used to write data in the specified MCU memory.
#define BL_MEM_WRITE 0x57
// This command is used to control read/write protection on different sections
#define BL_EN_RW_PROTECT 0x58
// This command is used to read data from specified MCU memory
#define BL_MEM_READ 0x59
// This command is used to read all the sector protection status
#define BL_READ_SECTOR_STATUS 0x5A
// This command is used to read OTP contents
#define BL_OTP_READ 0x5B
//This command is used disable all sector read/write protection
#define BL_DIS_RW_PROTECT			0x5C

/* ACK and NACK Bytes*/
#define BL_ACK 0xA5
#define BL_NACK 0x7F

/*
 * Bootloader version
 */
#define BL_VERSION 0x10

/*CRC*/
#define VERIFY_CRC_FAIL    1
#define VERIFY_CRC_SUCCESS 0

/*ADDRESS VALIDY*/
#define ADDR_VALID 0x00
#define ADDR_INVALID 0x01

#define INVALID_SECTOR  0x04

/*
 * Some Start and end addresses of different memories inside the
 * STM32F446xx
 */
#define SRAM1_SIZE			112*1024 //112 KB
#define SRAM1_END			(SRAM1_BASE + SRAM1_SIZE)
#define SRAM2_SIZE			16*1024 // 16KB
#define SRAM2_END			(SRAM2_BASE + SRAM2_SIZE)
#define FLASH_SIZE			512*1024 // 512KB
#define BACKUP_SRAM_BASE	BKPSRAM_BASE
#define BACKUP_SRAM_SIZE	4*1024
#define BACKUP_SRAM_END		(BACKUP_SRAM_BASE + BACKUP_SRAM_SIZE)


/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void bootloader_jump_to_user_app(void);
void bootloader_uart_read_data(void);
void bootloader_handle_getver_cmd(uint8_t *buffer);
void bootloader_handle_gethelp_cmd(uint8_t *buffer);
void bootloader_handle_getcid_cmd(uint8_t *buffer);
void bootloader_handle_getrdp_cmd(uint8_t *buffer);
void bootloader_handle_goto_cmd(uint8_t *buffer);
void bootloader_handle_flash_erase_cmd(uint8_t *buffer);
void bootloader_handle_mem_write_cmd(uint8_t *buffer);
void bootloader_handle_en_rw_protect_cmd(uint8_t *buffer);
void bootloader_handle_dis_rw_protect_cmd(uint8_t *buffer);
void bootloader_handle_mem_read_cmd(uint8_t *buffer);
void bootloader_handle_read_sector_status_cmd(uint8_t *buffer);
void bootloader_handle_read_otp_cmd(uint8_t *buffer);
void bootloader_send_ack(uint8_t command_code, uint8_t follow_len);
void bootloader_send_nack(void);
void bootloader_uart_write_data(uint8_t *pData, uint8_t len);
uint8_t bootloader_verify_crc(uint8_t* pData, uint32_t len, uint32_t crc_host);
uint8_t get_bootloader_version(void);
uint16_t get_mcu_chip_id(void);
uint8_t get_flash_rdp_level(void);
uint8_t verify_address(uint32_t go_address);
uint8_t execute_flash_erase(uint8_t sector_number, uint8_t number_of_sector);
uint8_t execute_mem_write(uint8_t *pBuffer, uint32_t mem_address, uint32_t len);
uint8_t configure_flash_sector_rw_protection(uint8_t sector_details, uint8_t protection_mode, uint8_t disable);
uint16_t read_OB_rw_protection_status(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
