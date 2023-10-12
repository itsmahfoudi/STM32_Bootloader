
# ABOUT
 This repository contains bare-metal firmware for a Cortex-M4 STM32 microcontroller (STM32F446RE in my case), using open source tools and libraries such as GCC for ARM and libopencm3. The project purpuse is to build a bootloader for performing firmware updates over UART/USB.

# Repo setup 
### Clone the repo
git clone git@github.com:lowbyteproductions/bare-metal-series.git
cd bare-metal-series

### Initialise the submodules (libopencm3)
git submodule init
git submodule update

### Build libopencm3
cd libopencm3
make
cd ..

### Build the main application firmware
cd app
make

# DEBUGGER
In this project i've used the ST-Link DEBUGGER and Programmer attached to the board, in my case i've used NUCLEO STM32F446RE 
