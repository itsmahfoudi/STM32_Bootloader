
# ABOUT
 This repository contains bare-metal firmware for a Cortex-M4 STM32 microcontroller (STM32F446RE in my case), using open source tools and libraries such as GCC for ARM and STM HAL. The project purpuse is to build a bootloader for performing firmware updates over UART.

# DEBUGGER
In this project i've used the ST-Link DEBUGGER and Programmer attached to the board, in my case i've used NUCLEO STM32F446RE 

# The supported bootloader commands

| Host Command          | Command Code | Bootloader Reply                            | Notes                                                                        |
| --------------------- | ------------ | ------------------------------------------- | ---------------------------------------------------------------------------- |
| BL_GET_VER            | 0x51         | Bootloader version number (1 byte)          | This command is used to read the bootloader version from the MCU.            |
| BL_GET_HELP           | 0x52         | All supported command code (10 bytes)       | This command is used to get what commands are supported.                     |
| BL_GET_CID            | 0x53         | Chip identification (2 bytes)               | This command is used to get MCU chip identification number.                  |
| BL_GET_RDP_STATUS     | 0x54         | Flash read protection level (1 byte)        | This command is used to get Flash read protection level.                     |
| BL_GO_TO_ADDR         | 0x55         | Success or Error code (1 byte)              | This command is used to jump the bootloader to the specified address.        |
| BL_FLASH_ERASE        | 0x56         | Success or Error Code (1 byte)              | This command is used to erase sectors of the user flash.                     |
| BL_MEM_WRITE          | 0x57         | Success or Error Code (1 byte)              | This command is used to write data to the specified MCU memory.              |
| BL_EN_RW_PROTECT      | 0x58         | Success or Error Code (1 byte)              | This command is used to enable read/write protection on different sections.  |
| BL_DIS_RW_PROTECT     | 0x5C         |  Success or Error Code (1 byte)             | This command is used to disable read/write protection on different sections. |
| BL_READ_SECTOR_STATUS | 0x5A         | All sectors status (2 bytes)                | This command is used to read all the sector protection status.               |
