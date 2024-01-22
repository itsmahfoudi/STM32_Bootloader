#include "main.h"

/* Private variables *********************************************/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

uint8_t supported_commands[] = {
								BL_GET_VER,
								BL_GET_HELP,
								BL_GET_CID,
								BL_GET_RDP_STATUS,
								BL_GO_TO_ADDR,
								BL_FLASH_ERASE,
								BL_MEM_WRITE,
								BL_READ_SECTOR_STATUS};

#define D_UART &huart3
#define C_UART &huart2

char dataToSend[] = "Hello from STM32 nucleo board.\r\n";
/* USER CODE BEGIN PV*/

/* USER CODE END PV */

/* Private functions declarations ********************************/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void printmsg(char *format, ...);
/* USER CODE BEGIN PFP*/

/* USER CODE END PFP */
/* Private USER code ********************************/
/* USER CODE BEGIN 0*/
#define BL_RX_LEN 200
uint8_t bl_rx_buffer[BL_RX_LEN];

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */

int main(void) {
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the
   *
 *

   * * Systick.

   */
  HAL_Init();

  /* USER CODE BEGIN SysInit */
  /* Configure the system clock */
  SystemClock_Config();
  /* USER CODE END SysInit */

  /* USER CODE BEGIN Init */
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CRC_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();

  if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET) {
    printmsg("BL_DEBUG_MSG: Button is pressed... Jumping to BL mode.\n");
    bootloader_uart_read_data();
  } else {
    printmsg("BL_DEBUG_MSG: Button is not pressed... Jumping to User app.\n");
    bootloader_jump_to_user_app();
  }

  return 0;
}

/**
 * @brief Simple printf implementation
 * @retval None
 */
static void printmsg(char *format, ...) {
  char str[100];
  va_list args;
  va_start(args, format);
  vsprintf(str, format, args);
  HAL_UART_Transmit(D_UART, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
  va_end(args);
}

/**
 * @brief Function to jump to the 3 section of the flash where the user app
 * should exist.
 * @retval None
 */
void bootloader_jump_to_user_app(void) {
  // function pointer to hold the address of the reset handler of the user app.
  void (*app_reset_handler)(void);
  printmsg("BL_DEBUG_MSG: bootloader_jump_to_user_app\n");

  // 1. Configure the MSP
  uint32_t msp_value = *(volatile uint32_t *)FLASH_SECTOR2_BASE_ADDRESS;
  printmsg("BL_DEBUG_MSG: MSP value : %#x\n", msp_value);

  // setting the value of MSP
  __set_MSP(msp_value);

  // 2. Fetch the reset handler of the user app. existing in flash sector 2
  uint32_t reset_handler_address =
      *(volatile uint32_t *)(FLASH_SECTOR2_BASE_ADDRESS + 4);
  app_reset_handler = (void *)reset_handler_address;
  printmsg("BL_DEBUG_MSG: app reset handler addr : %#x\n", app_reset_handler);

  // 3. jump to the reset handler of the application
  app_reset_handler();
}

/**
 * @brief Function to listen on the uart port and execute the received commands.
 * @retval None
 */
void bootloader_uart_read_data(void) {
  uint8_t rcv_len = 0;
  while (1) {
    memset(bl_rx_buffer, 0, 200);
    HAL_UART_Receive(C_UART, bl_rx_buffer, 1, HAL_MAX_DELAY);
    rcv_len = bl_rx_buffer[0];
    HAL_UART_Receive(C_UART, &bl_rx_buffer[1], rcv_len, HAL_MAX_DELAY);
    switch (bl_rx_buffer[1]) {
    case BL_GET_VER:
      bootloader_handle_getver_cmd(bl_rx_buffer);
      break;
    case BL_GET_HELP:
      bootloader_handle_gethelp_cmd(bl_rx_buffer);
      break;
    case BL_GET_CID:
      bootloader_handle_getcid_cmd(bl_rx_buffer);
      break;
    case BL_GET_RDP_STATUS:
      bootloader_handle_getrdp_cmd(bl_rx_buffer);
      break;
    case BL_GO_TO_ADDR:
      bootloader_handle_goto_cmd(bl_rx_buffer);
      break;
    case BL_FLASH_ERASE:
      bootloader_handle_flash_erase_cmd(bl_rx_buffer);
      break;
    case BL_MEM_WRITE:
      bootloader_handle_mem_write_cmd(bl_rx_buffer);
      break;
    case BL_ENDIS_RW_PROTECT:
      bootloader_handle_endis_rw_protect_cmd(bl_rx_buffer);
      break;
    case BL_MEM_READ:
      bootloader_handle_mem_read_cmd(bl_rx_buffer);
      break;
    case BL_READ_SECTOR_STATUS:
      bootloader_handle_read_sector_status_cmd(bl_rx_buffer);
      break;
    case BL_OTP_READ:
      bootloader_handle_read_otp_cmd(bl_rx_buffer);
      break;
    default:
      printmsg("BL_DEBUG_MSG: Invalid Command code received form host!!\n");
      break;
    }
  }
}

/**
 * @brief Function to write on the C_UART.
 * @retval None
 */
void bootloader_uart_write_data(uint8_t *pData, uint8_t len) {
    HAL_UART_Transmit(C_UART, pData,len, HAL_MAX_DELAY);
}



/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters

   * *


   * * * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void) {

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
  if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_USART3_UART_Init(void) {

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
  if (HAL_UART_Init(&huart3) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 *
 *
 *
 * @retval
 * None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number



 * * * *
 * where the assert_param error has occurred.
 * @param  file: pointer

 * * to
 * the
 * source file name
 * @param  line: assert_param error line
 *
 * source
 * number
 *
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line

 *


   * * * number, ex: printf("Wrong parameters value: file %s on line %d\r\n",

   * *
   * file,

   * line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/*
 * *********************Bootloader commands implementation.******************/
/**
 * @brief Helper function to handle the BL_GET_VER Command
 * @param pointer to the buffer where the operands of the command resides.
 * @retval None
 */
void bootloader_handle_getver_cmd(uint8_t *bl_rx_buffer) {
    uint8_t bl_version;
    printmsg("BL_DEBUG_MSG : bootloader_handle_getver_cmd\n");
    //total length of the command packet
    uint32_t command_packet_len = bl_rx_buffer[0] + 1;

    //extract the crc32 sent by the host
    uint32_t host_crc = *((uint32_t*) (bl_rx_buffer + command_packet_len-4));
    if (! bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len-4, host_crc )) {
        printmsg("BL_DEBUG_MSG : checksum success !!\n");
        //Send the acknowledgment to the host.
        bootloader_send_ack(bl_rx_buffer[0],1);
        bl_version = get_bootloader_version();
        printmsg("BL_DEBUG_MSG:BL_VER : %d %#x\n",bl_version, bl_version);
        //Send the value of the bootloader version to the host.
        bootloader_uart_write_data(&bl_version,1);
    } else {
        printmsg("BL_DEBUG_MSG:checksum fail!!\n");
        //Send the non acknowledgment to the host.
        bootloader_send_nack();
    }
}

/**
 * @brief Helper function to handle the BL_GET_HELP Command
 * @param pointer to the buffer where the operands of the command resides.
 * @retval None
 */
void bootloader_handle_gethelp_cmd(uint8_t *buffer) {
	printmsg("BL_DEBUG_MSG: bootloader_handler_gethelp_cmd\n");

	//Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0] + 1;

	//extract the crc32 sent by the host
	uint32_t host_crc = *((uint32_t *) (bl_rx_buffer+command_packet_len-4));

	if (! bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len-4, host_crc)) {
		printmsg("BL_DEBUG_MSG:Checksum success !!\n");
		bootloader_send_ack(bl_rx_buffer[0], sizeof(supported_commands));
		bootloader_uart_write_data(supported_commands, sizeof(supported_commands));
	} else {
		printmsg("BL_DEBUG_MSG: checksum fail !!");
		bootloader_send_nack();
  }
}
/**
 * @brief Helper function to handle the BL_GET_CID Command
 * @param pointer to the buffer where the operands of the command resides.
 * @retval None
 */
void bootloader_handle_getcid_cmd(uint8_t *buffer) {
	uint16_t bl_cid_num = 0;
	printmsg("BL_DEBUG_MSG: bootloader_handle_getcid_cmd\n");

	//Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1;

	//extract the CRC32 sent by the host
	uint32_t host_crc = *((uint32_t*) (bl_rx_buffer + command_packet_len - 4));

	if (!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len-4, host_crc)) {
		printmsg("BL_DEBUG_MSG: checksum success !!\n");
		bootloader_send_ack(bl_rx_buffer[0], command_packet_len);
		bl_cid_num = get_mcu_chip_id();
		printmsg("BL_DEBUG_MSG: MCU id : %d %#x !!!",bl_cid_num,bl_cid_num);
		bootloader_uart_write_data((uint8_t*)&bl_cid_num, 2);
	} else {
		printmsg("BL_DEBUG_MSG: checksum fail !!\n");
		bootloader_send_nack();
	}
}

/**
 * @brief Helper function to handle the BL_GET_RDP_STATUS Command
 * @param pointer to the buffer where the operands of the command resides.
 * @retval None
 */
void bootloader_handle_getrdp_cmd(uint8_t *buffer) {
	uint8_t rdp_level = 0x00;
	printmsg("BL_DEBUG_MSG: bootloader_handle_getrdp_cmd\n");

	//Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1;

	//extract the CRC32 sent by the host
	uint32_t host_crc = *((uint32_t*) (bl_rx_buffer + command_packet_len - 4));

	if (!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len-4, host_crc)) {
		printmsg("BL_DEBUG_MSG: checksum success !!\n");
		bootloader_send_ack(bl_rx_buffer[0], command_packet_len);
		rdp_level = get_flash_rdp_level();
		printmsg("BL_DEBUG_MSG: RDP Level : %d %#x !!!",rdp_level,rdp_level);
		bootloader_uart_write_data(&rdp_level, 1);
	} else {
		printmsg("BL_DEBUG_MSG: checksum fail !!\n");
		bootloader_send_nack();
	}
}

/**
 * @brief Helper function to handle the BL_GO_TO_ADDR Command
 * @param pointer to the buffer where the operands of the command resides.
 * @retval None
 */
void bootloader_handle_goto_cmd(uint8_t *buffer) {
	uint32_t go_address = 0;
	uint8_t addr_valid = ADDR_VALID;
	uint8_t addr_invalid = ADDR_INVALID;

	printmsg("BL_DEBUG_MSG:bootloader_handle_go_cmd\n");

	//Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1;

	//extract the CRC32 sent by the host
	uint32_t host_crc = *((uint32_t*) (bl_rx_buffer + command_packet_len - 4));

	if (!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len-4, host_crc)) {
		printmsg("BL_DEBUG_MSG: checksum success !!\n");
		bootloader_send_ack(bl_rx_buffer[0], 1);
		go_address = *((uint32_t*)&bl_rx_buffer[2]);
		printmsg("BL_DEBUG_MSG: GO Addr : %#x !!!",go_address);
		if (verify_address(go_address) == ADDR_VALID) {
			//Confirm to the host.
			bootloader_uart_write_data(&addr_valid, 1);

			/*
			 * Jump to "go_address".
			 * Host must ensure that the valid code is present at
			 * the specified address, It's not the duty of
			 * bootloader.
			 */

			go_address += 1;
			void(*lets_jump)(void) = (void*)go_address;

			printmsg("BL_DEBUG_MSG: Jumping to go address! \n");

			lets_jump();
		} else {
			printmsg("BL_DEBUG_MSG: Go address invalid!!!\n");
			bootloader_uart_write_data(&addr_invalid, 1);
		}
	} else {
		printmsg("BL_DEBUG_MSG: checksum fail !!\n");
		bootloader_send_nack();
	}
}

/**
 * @brief Helper function to handle the BL_FLASH_ERASE Command
 * @param pointer to the buffer where the operands of the command resides.
 * @retval None
 */
void bootloader_handle_flash_erase_cmd(uint8_t *buffer) {
	uint8_t erase_status = 0x00;
	printmsg("BL_DEBUG_MSG: bootloader_handle_flash_erase_cmd\n");

	//Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1;

	//extract the CRC32 sent by the host
	uint32_t host_crc = *((uint32_t*) (bl_rx_buffer + command_packet_len - 4));

	if (!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len-4, host_crc)) {
		printmsg("BL_DEBUG_MSG: checksum success !!\n");
		bootloader_send_ack(bl_rx_buffer[0], 1);
		printmsg("BL_DEBUG_MSG: Initial sector : %d N_ofsectors %d\n",bl_rx_buffer[2],bl_rx_buffer[3]);

		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
		erase_status = execute_flash_erase(bl_rx_buffer[2], bl_rx_buffer[3]);
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);

		printmsg("BL_DEBUG_MSG:flash erase status: %#x\n",erase_status);

		bootloader_uart_write_data(&erase_status, 1);
	} else {
		printmsg("BL_DEBUG_MSG: checksum fail !!\n");
		bootloader_send_nack();
	}
}

/**
 * @brief Helper function to handle the BL_MEM_WRITE Command
 * @param pointer to the buffer where the operands of the command resides.
 * @retval None
 */
void bootloader_handle_mem_write_cmd(uint8_t *buffer) {}

/**
 * @brief Helper function to handle the BL_ENDIS_RW_PROTECT Command
 * @param pointer to the buffer where the operands of the command resides.
 * @retval None
 */
void bootloader_handle_endis_rw_protect_cmd(uint8_t *buffer) {}

/**
 * @brief Helper function to handle the BL_MEM_READ Command
 * @param pointer to the buffer where the operands of the command resides.
 * @retval None
 */
void bootloader_handle_mem_read_cmd(uint8_t *buffer) {}

/**
 * @brief Helper function to handle the BL_READ_SECTOR_STATUS Command
 * @param pointer to the buffer where the operands of the command resides.
 * @retval None
 */
void bootloader_handle_read_sector_status_cmd(uint8_t *buffer) {}

/**
 * @brief Helper function to handle the BL_OTP_READ Command
 * @param pointer to the buffer where the operands of the command resides.
 * @retval None
 */
void bootloader_handle_read_otp_cmd(uint8_t *buffer) {}


/**
 * @brief Helper function to send acknowledgment to the host
 * @param command code.
 *        length of the items to follow.
 * @retval None
 */
void bootloader_send_ack(uint8_t command_code, uint8_t follow_len) {
    uint8_t ack_buf[2];
    ack_buf[0] = BL_ACK;
    ack_buf[1] = follow_len;
    HAL_UART_Transmit(C_UART,  ack_buf, 2, HAL_MAX_DELAY);
}

/**
 * @brief Helper function to send acknowledgment to the host
 * @param None
 * @retval None
 */
void bootloader_send_nack(void) {
    uint8_t nack = BL_NACK;
    HAL_UART_Transmit(C_UART, &nack, 1, HAL_MAX_DELAY);
}


/**
 * @brief Helper function to verify the crc value received by the host.
 * @param0 : pointer to the data to which the crc will be calculated
 * @param1 : length of data to which the crc will be calculated
 * @param2 : the value of the crc received by the host.
 * @retval None
 */
uint8_t bootloader_verify_crc(uint8_t* pData, uint32_t len, uint32_t crc_host) {
    uint32_t uwCRCValue = 0xFF;
    for (uint32_t i = 0; i < len; i++) {
        uint32_t i_data = pData[i];
        uwCRCValue = HAL_CRC_Accumulate(&hcrc,&i_data,1);
    }
    if (uwCRCValue == crc_host) {
        return VERIFY_CRC_SUCCESS;
    }
    return VERIFY_CRC_FAIL;
}

/**
 * @brief Helper function to send acknowledgment to the host
 * @param None
 * @retval unsigned byte containing the version of the bootloader.
 */
uint8_t get_bootloader_version(void) {
    return (uint8_t)BL_VERSION;
}

/**
 * @brief Helper function to send acknowledgment to the host
 * @param None
 * @retval unsigned 2byte containing the chip or device identifier.
 */
uint16_t get_mcu_chip_id(void) {
	/*
	 * The STM32F446xx MCUs integrate an MCU ID code. This ID identifies the ST MCU PartNumber
	 * and the die revision. It is part of the DBG_MCU component and is mapped on the
	 * external PPB bus (see Section 33.16 on page 1304). This code is accessible using the
	 * JTAG debug pCat.2port (4 to 5 pins) or the SW debug port (two pins) or by the user software.
	 * it is even accessible while the MCU is under system reset.
	 */
	uint16_t cid;
	cid = (uint16_t) (DBGMCU->IDCODE) & (0x0FFF);
	return cid;
}

/**
 * @brief Helper function to get read protection level of flash
 * @param None
 * @retval unsigned byte indicating the rdp level.
 */
uint8_t get_flash_rdp_level(void) {
	uint8_t rdp_status = 0;
	FLASH_OBProgramInitTypeDef ob_handle;
	HAL_FLASHEx_OBGetConfig(&ob_handle);
	rdp_status = (uint8_t) ob_handle.RDPLevel;
}

uint8_t execute_flash_erase(uint8_t sector_number, uint8_t number_of_sector) {
	//we have totally 8 sectors in STM32F446RE mcu .. sector[0 to 7]
	//number_of_sector has to be in the range of 0 to 7
	// if sector_number = 0xff , that means mass erase !
	//Code needs to modified if your MCU supports more flash sectors
	FLASH_EraseInitTypeDef flashErase_handle;
	uint32_t sectorError;
	HAL_StatusTypeDef status;

	if (number_of_sector > 8) {
		return INVALID_SECTOR;
	}
	if ((sector_number == 0xFF) || (sector_number <= 7)) {
		if (sector_number == (uint8_t) 0xFF) {
			//The erase operation is mass erase
			flashErase_handle.TypeErase = FLASH_TYPEERASE_MASSERASE;
		} else {
			uint8_t remaining_sector = 8 - sector_number;
			if (number_of_sector > remaining_sector) {
				number_of_sector = remaining_sector;
			}
			flashErase_handle.TypeErase = FLASH_TYPEERASE_SECTORS;
			flashErase_handle.Sector = sector_number;
			flashErase_handle.NbSectors = number_of_sector;
		}
		flashErase_handle.Banks = FLASH_BANK_1;

		/*Get access to get control of the flash registers.*/
		HAL_FLASH_Unlock();
		flashErase_handle.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		status = (uint8_t)HAL_FLASHEx_Erase(&flashErase_handle, &sectorError);
		HAL_FLASH_Lock();

		return status;
	}
	return INVALID_SECTOR;

}
