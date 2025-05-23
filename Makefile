# This file was automagically generated by UNER - CP3

###############################################################################
PROJECTPATH = C:/Users/tadeo/OneDrive/Documentos/A_Facultad/Mecanica_Racional/Tercer_trabajo/Actividad3Chury
COMPILERDIR = C:/Users/tadeo/OneDrive/Documentos/A_Facultad/Mecanica_Racional/MBEDGCC_VSC23/gcc-arm-none-eabi-6-2017-q2-update-win32/bin
MBEDPATH = C:/Users/tadeo/OneDrive/Documentos/A_Facultad/Mecanica_Racional/MBEDGCC_VSC23

###############################################################################
# Boiler-plate

# cross-platform directory manipulation
ifeq ($(shell echo $$OS),$$OS)
    MAKEDIR = if not exist "$(1)" mkdir "$(1)"
    RM = rmdir /S /Q "$(1)"
else
    MAKEDIR = '$(SHELL)' -c "mkdir -p "$(1)\""
    RM = '$(SHELL)' -c "rm -rf "$(1)\""
endif

OBJDIR := BUILD
# Move to the build directory
ifeq (,$(filter $(OBJDIR),$(notdir $(CURDIR))))
.SUFFIXES:
mkfile_path := $(abspath $(lastword $(MAKEFILE_LIST)))
MAKETARGET = '$(MAKE)' --no-print-directory -C $(OBJDIR) -f '$(mkfile_path)' \
		'SRCDIR=$(CURDIR)' $(MAKECMDGOALS)
.PHONY: $(OBJDIR) clean
all:
	+@$(call MAKEDIR,$(OBJDIR))
	+@$(MAKETARGET)
$(OBJDIR): all
Makefile : ;
% :: $(OBJDIR) ; :
clean :
	$(call RM,$(OBJDIR))

else

# trick rules into thinking we are in the root, when we are in the bulid dir
# VPATH = ..

VPATH = $(PROJECTPATH)

# Boiler-plate
###############################################################################
# Project settings

# PROJECT := ProjectOut
PROJECT := Actividad3Chury


# Project settings
###############################################################################
# Objects and Paths

# OBJECTS += main.o
# OBJECTS += $(patsubst $(PROJECTPATH)/%.c,%.o,$(wildcard $(PROJECTPATH)/*.c))
OBJECTS += $(patsubst $(PROJECTPATH)/%.cpp,%.o,$(wildcard $(PROJECTPATH)/*.cpp))

SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/PeripheralPins.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/analogin_api.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/can_api.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/cmsis_nvic.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/gpio_api.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/gpio_irq_api.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/hal_tick_16b.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/hal_tick_32b.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/i2c_api.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/lp_ticker.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/mbed_board.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/mbed_overrides.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/pinmap.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/port_api.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/pwmout_api.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/retarget.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/rtc_api.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/serial_api.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/sleep.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/spi_api.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/startup_stm32f103xb.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_adc.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_adc_ex.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_can.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_cec.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_cortex.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_crc.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_dac.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_dac_ex.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_dma.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_eth.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_flash.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_flash_ex.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_gpio.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_gpio_ex.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_hcd.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_i2c.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_i2s.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_irda.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_iwdg.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_nand.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_nor.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_pccard.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_pcd.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_pcd_ex.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_pwr.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_rcc.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_rcc_ex.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_rtc.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_rtc_ex.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_sd.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_smartcard.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_spi.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_spi_ex.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_sram.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_tim.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_tim_ex.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_uart.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_usart.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_hal_wwdg.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_ll_crc.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_ll_exti.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_ll_fsmc.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_ll_gpio.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_ll_pwr.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_ll_rcc.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_ll_sdmmc.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_ll_usb.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm32f1xx_ll_utils.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/stm_spi_api.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/system_stm32f1xx.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/trng_api.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/us_ticker_16b.o
SYS_OBJECTS += $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/us_ticker_32b.o

INCLUDE_PATHS += -I$(MBEDPATH)/.
INCLUDE_PATHS += -I$(MBEDPATH)//usr/src/mbed-sdk
INCLUDE_PATHS += -I$(MBEDPATH)/mbed
INCLUDE_PATHS += -I$(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB
INCLUDE_PATHS += -I$(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TARGET_STM
INCLUDE_PATHS += -I$(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TARGET_STM/TARGET_STM32F1
INCLUDE_PATHS += -I$(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TARGET_STM/TARGET_STM32F1/TARGET_NUCLEO_F103RB
INCLUDE_PATHS += -I$(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TARGET_STM/TARGET_STM32F1/TARGET_NUCLEO_F103RB/device
INCLUDE_PATHS += -I$(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TARGET_STM/TARGET_STM32F1/device
INCLUDE_PATHS += -I$(MBEDPATH)/mbed/drivers
INCLUDE_PATHS += -I$(MBEDPATH)/mbed/hal
INCLUDE_PATHS += -I$(MBEDPATH)/mbed/platform

LIBRARY_PATHS := -L$(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM 
LIBRARIES := -lmbed 
LINKER_SCRIPT ?= $(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TOOLCHAIN_GCC_ARM/STM32F103XB.ld

# Objects and Paths
###############################################################################
# Tools and Flags

AS      = $(COMPILERDIR)/arm-none-eabi-gcc
CC      = $(COMPILERDIR)/arm-none-eabi-gcc
CPP     = $(COMPILERDIR)/arm-none-eabi-g++
LD      = $(COMPILERDIR)/arm-none-eabi-gcc
ELF2BIN = $(COMPILERDIR)/arm-none-eabi-objcopy
ELF2SIZE = $(COMPILERDIR)/arm-none-eabi-size
PREPROC = $(COMPILERDIR)/arm-none-eabi-cpp -E -P -Wl,--gc-sections -Wl,--wrap,main -Wl,--wrap,_malloc_r -Wl,--wrap,_free_r -Wl,--wrap,_realloc_r -Wl,--wrap,_memalign_r -Wl,--wrap,_calloc_r -Wl,--wrap,exit -Wl,--wrap,atexit -Wl,-n --specs=nano.specs -Wl,--wrap,printf -Wl,--wrap,sprintf -Wl,--wrap,snprintf -Wl,--wrap,vprintf -Wl,--wrap,vsnprintf -Wl,--wrap,fprintf -Wl,--wrap,vfprintf -mcpu=cortex-m3 -mthumb -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -fmessage-length=0 -fno-exceptions -ffunction-sections -fdata-sections -funsigned-char -MMD -fomit-frame-pointer -Os -g -DMBED_TRAP_ERRORS_ENABLED=1 -DMBED_RTOS_SINGLE_THREAD -D__NEWLIB_NANO -DMBED_MINIMAL_PRINTF -mcpu=cortex-m3 -mthumb -DMBED_ROM_START=0x8000000 -DMBED_ROM_SIZE=0x10000 -DMBED_RAM_START=0x20000000 -DMBED_RAM_SIZE=0x5000 -DMBED_BOOT_STACK_SIZE=1024 -DXIP_ENABLE=0


C_FLAGS += -c
C_FLAGS += -std=gnu11
C_FLAGS += -include
C_FLAGS += $(PROJECTPATH)/mbed_config.h
C_FLAGS += -DDEVICE_ANALOGIN=1
C_FLAGS += -DDEVICE_INTERRUPTIN=1
C_FLAGS += -DTRANSACTION_QUEUE_SIZE_SPI=2
C_FLAGS += -DTARGET_CORTEX
C_FLAGS += -DDEVICE_SERIAL_FC=1
C_FLAGS += -D__CORTEX_M3
C_FLAGS += -DTARGET_MCU_STM32
C_FLAGS += -DTARGET_FF_MORPHO
C_FLAGS += -DDEVICE_WATCHDOG=1
C_FLAGS += -DTARGET_NUCLEO_F103RB
C_FLAGS += -DDEVICE_I2C_ASYNCH=1
C_FLAGS += -DTARGET_CORTEX_M
C_FLAGS += -DDEVICE_SPI_ASYNCH=1
C_FLAGS += -DTARGET_RELEASE
C_FLAGS += -DDEVICE_I2CSLAVE=1
C_FLAGS += -DDEVICE_RESET_REASON=1
C_FLAGS += -DDEVICE_PORTIN=1
C_FLAGS += -DDEVICE_SERIAL_ASYNCH=1
C_FLAGS += -DDEVICE_STDIO_MESSAGES=1
C_FLAGS += -DTARGET_NAME=NUCLEO_F103RB
C_FLAGS += -D__MBED_CMSIS_RTOS_CM
C_FLAGS += -DTARGET_FF_ARDUINO
C_FLAGS += -DTARGET_STM
C_FLAGS += -DTARGET_STM32F103xB
C_FLAGS += -DTARGET_LIKE_CORTEX_M3
C_FLAGS += -DUSE_HAL_DRIVER
C_FLAGS += -DDEVICE_FLASH=1
C_FLAGS += -DTARGET_STM32F1
C_FLAGS += -DARM_MATH_CM3
C_FLAGS += -DDEVICE_PORTINOUT=1
C_FLAGS += -DDEVICE_RTC=1
C_FLAGS += -DDEVICE_SLEEP=1
C_FLAGS += -DDEVICE_USTICKER=1
C_FLAGS += -DDEVICE_PWMOUT=1
C_FLAGS += -D__MBED__=1
C_FLAGS += -DSTM32F103xB
C_FLAGS += -DUSE_FULL_LL_DRIVER
C_FLAGS += -DTARGET_LIKE_MBED
C_FLAGS += -DDEVICE_SPI=1
C_FLAGS += -D__CMSIS_RTOS
C_FLAGS += -DDEVICE_SERIAL=1
C_FLAGS += -DDEVICE_SPISLAVE=1
C_FLAGS += -DDEVICE_I2C=1
C_FLAGS += -DTARGET_MCU_STM32_BAREMETAL
C_FLAGS += -DTOOLCHAIN_GCC
C_FLAGS += -DDEVICE_PORTOUT=1
C_FLAGS += -DTOOLCHAIN_GCC_ARM
C_FLAGS += -DDEVICE_CAN=1
C_FLAGS += -DTARGET_M3
C_FLAGS += -DMBED_BUILD_TIMESTAMP=1655156899.5259805
C_FLAGS += -DTARGET_MCU_STM32F103xB
C_FLAGS += -include
C_FLAGS += $(PROJECTPATH)/mbed_config.h
C_FLAGS += -c
C_FLAGS += -std=gnu11
C_FLAGS += -Wall
C_FLAGS += -Wextra
C_FLAGS += -Wno-unused-parameter
C_FLAGS += -Wno-missing-field-initializers
C_FLAGS += -fmessage-length=0
C_FLAGS += -fno-exceptions
C_FLAGS += -ffunction-sections
C_FLAGS += -fdata-sections
C_FLAGS += -funsigned-char
C_FLAGS += -MMD
C_FLAGS += -fomit-frame-pointer
C_FLAGS += -Os
C_FLAGS += -g
C_FLAGS += -DMBED_TRAP_ERRORS_ENABLED=1
C_FLAGS += -DMBED_RTOS_SINGLE_THREAD
C_FLAGS += -D__NEWLIB_NANO
C_FLAGS += -DMBED_MINIMAL_PRINTF
C_FLAGS += -mcpu=cortex-m3
C_FLAGS += -mthumb
C_FLAGS += -DMBED_ROM_START=0x8000000
C_FLAGS += -DMBED_ROM_SIZE=0x20000
C_FLAGS += -DMBED_RAM_START=0x20000000
C_FLAGS += -DMBED_RAM_SIZE=0x5000

CXX_FLAGS += -c
CXX_FLAGS += -std=gnu++14
CXX_FLAGS += -fno-rtti
CXX_FLAGS += -Wvla
CXX_FLAGS += -include
CXX_FLAGS += $(PROJECTPATH)/mbed_config.h
CXX_FLAGS += -DDEVICE_ANALOGIN=1
CXX_FLAGS += -DDEVICE_INTERRUPTIN=1
CXX_FLAGS += -DTRANSACTION_QUEUE_SIZE_SPI=2
CXX_FLAGS += -DTARGET_CORTEX
CXX_FLAGS += -DDEVICE_SERIAL_FC=1
CXX_FLAGS += -D__CORTEX_M3
CXX_FLAGS += -DTARGET_MCU_STM32
CXX_FLAGS += -DTARGET_FF_MORPHO
CXX_FLAGS += -DDEVICE_WATCHDOG=1
CXX_FLAGS += -DTARGET_NUCLEO_F103RB
CXX_FLAGS += -DDEVICE_I2C_ASYNCH=1
CXX_FLAGS += -DTARGET_CORTEX_M
CXX_FLAGS += -DDEVICE_SPI_ASYNCH=1
CXX_FLAGS += -DTARGET_RELEASE
CXX_FLAGS += -DDEVICE_I2CSLAVE=1
CXX_FLAGS += -DDEVICE_RESET_REASON=1
CXX_FLAGS += -DDEVICE_PORTIN=1
CXX_FLAGS += -DDEVICE_SERIAL_ASYNCH=1
CXX_FLAGS += -DDEVICE_STDIO_MESSAGES=1
CXX_FLAGS += -DTARGET_NAME=NUCLEO_F103RB
CXX_FLAGS += -D__MBED_CMSIS_RTOS_CM
CXX_FLAGS += -DTARGET_FF_ARDUINO
CXX_FLAGS += -DTARGET_STM
CXX_FLAGS += -DTARGET_STM32F103xB
CXX_FLAGS += -DTARGET_LIKE_CORTEX_M3
CXX_FLAGS += -DUSE_HAL_DRIVER
CXX_FLAGS += -DDEVICE_FLASH=1
CXX_FLAGS += -DTARGET_STM32F1
CXX_FLAGS += -DARM_MATH_CM3
CXX_FLAGS += -DDEVICE_PORTINOUT=1
CXX_FLAGS += -DDEVICE_RTC=1
CXX_FLAGS += -DDEVICE_SLEEP=1
CXX_FLAGS += -DDEVICE_USTICKER=1
CXX_FLAGS += -DDEVICE_PWMOUT=1
CXX_FLAGS += -D__MBED__=1
CXX_FLAGS += -DSTM32F103xB
CXX_FLAGS += -DUSE_FULL_LL_DRIVER
CXX_FLAGS += -DTARGET_LIKE_MBED
CXX_FLAGS += -DDEVICE_SPI=1
CXX_FLAGS += -D__CMSIS_RTOS
CXX_FLAGS += -DDEVICE_SERIAL=1
CXX_FLAGS += -DDEVICE_SPISLAVE=1
CXX_FLAGS += -DDEVICE_I2C=1
CXX_FLAGS += -DTARGET_MCU_STM32_BAREMETAL
CXX_FLAGS += -DTOOLCHAIN_GCC
CXX_FLAGS += -DDEVICE_PORTOUT=1
CXX_FLAGS += -DTOOLCHAIN_GCC_ARM
CXX_FLAGS += -DDEVICE_CAN=1
CXX_FLAGS += -DTARGET_M3
CXX_FLAGS += -DMBED_BUILD_TIMESTAMP=1655156899.5259805
CXX_FLAGS += -DTARGET_MCU_STM32F103xB
CXX_FLAGS += -include
CXX_FLAGS += $(PROJECTPATH)/mbed_config.h
CXX_FLAGS += -c
CXX_FLAGS += -std=gnu++14
CXX_FLAGS += -fno-rtti
CXX_FLAGS += -Wvla
CXX_FLAGS += -Wall
CXX_FLAGS += -Wextra
CXX_FLAGS += -Wno-unused-parameter
CXX_FLAGS += -Wno-missing-field-initializers
CXX_FLAGS += -fmessage-length=0
CXX_FLAGS += -fno-exceptions
CXX_FLAGS += -ffunction-sections
CXX_FLAGS += -fdata-sections
CXX_FLAGS += -funsigned-char
CXX_FLAGS += -MMD
CXX_FLAGS += -fomit-frame-pointer
CXX_FLAGS += -Os
CXX_FLAGS += -g
CXX_FLAGS += -DMBED_TRAP_ERRORS_ENABLED=1
CXX_FLAGS += -DMBED_RTOS_SINGLE_THREAD
CXX_FLAGS += -D__NEWLIB_NANO
CXX_FLAGS += -DMBED_MINIMAL_PRINTF
CXX_FLAGS += -mcpu=cortex-m3
CXX_FLAGS += -mthumb
CXX_FLAGS += -DMBED_ROM_START=0x8000000
CXX_FLAGS += -DMBED_ROM_SIZE=0x20000
CXX_FLAGS += -DMBED_RAM_START=0x20000000
CXX_FLAGS += -DMBED_RAM_SIZE=0x5000

ASM_FLAGS += -c
ASM_FLAGS += -x
ASM_FLAGS += assembler-with-cpp
ASM_FLAGS += -DUSE_FULL_LL_DRIVER
ASM_FLAGS += -D__CMSIS_RTOS
ASM_FLAGS += -DTRANSACTION_QUEUE_SIZE_SPI=2
ASM_FLAGS += -DUSE_HAL_DRIVER
ASM_FLAGS += -DARM_MATH_CM3
ASM_FLAGS += -D__CORTEX_M3
ASM_FLAGS += -DSTM32F103xB
ASM_FLAGS += -D__MBED_CMSIS_RTOS_CM
ASM_FLAGS += -I/usr/src/mbed-sdk
ASM_FLAGS += -I$(MBEDPATH)/mbed
ASM_FLAGS += -I$(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB
ASM_FLAGS += -I$(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TARGET_STM
ASM_FLAGS += -I$(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TARGET_STM/TARGET_STM32F1
ASM_FLAGS += -I$(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TARGET_STM/TARGET_STM32F1/TARGET_NUCLEO_F103RB
ASM_FLAGS += -I$(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TARGET_STM/TARGET_STM32F1/TARGET_NUCLEO_F103RB/device
ASM_FLAGS += -I$(MBEDPATH)/mbed/TARGET_NUCLEO_F103RB/TARGET_STM/TARGET_STM32F1/device
ASM_FLAGS += -I$(MBEDPATH)/mbed/drivers
ASM_FLAGS += -I$(MBEDPATH)/mbed/hal
ASM_FLAGS += -I$(MBEDPATH)/mbed/platform
ASM_FLAGS += -include
ASM_FLAGS += /filer/workspace_data/exports/b/b4e4e93a355cb51b46943e8445b4acd2/Test2022AI2/mbed_config.h
ASM_FLAGS += -c
ASM_FLAGS += -x
ASM_FLAGS += assembler-with-cpp
ASM_FLAGS += -Wall
ASM_FLAGS += -Wextra
ASM_FLAGS += -Wno-unused-parameter
ASM_FLAGS += -Wno-missing-field-initializers
ASM_FLAGS += -fmessage-length=0
ASM_FLAGS += -fno-exceptions
ASM_FLAGS += -ffunction-sections
ASM_FLAGS += -fdata-sections
ASM_FLAGS += -funsigned-char
ASM_FLAGS += -MMD
ASM_FLAGS += -fomit-frame-pointer
ASM_FLAGS += -Os
ASM_FLAGS += -g
ASM_FLAGS += -DMBED_TRAP_ERRORS_ENABLED=1
ASM_FLAGS += -DMBED_RTOS_SINGLE_THREAD
ASM_FLAGS += -D__NEWLIB_NANO
ASM_FLAGS += -DMBED_MINIMAL_PRINTF
ASM_FLAGS += -mcpu=cortex-m3
ASM_FLAGS += -mthumb


LD_FLAGS :=-Wl,--gc-sections -Wl,--wrap,main -Wl,--wrap,_malloc_r -Wl,--wrap,_free_r -Wl,--wrap,_realloc_r -Wl,--wrap,_memalign_r -Wl,--wrap,_calloc_r -Wl,--wrap,exit -Wl,--wrap,atexit -Wl,-n --specs=nano.specs -Wl,--wrap,printf -Wl,--wrap,sprintf -Wl,--wrap,snprintf -Wl,--wrap,fprintf -mcpu=cortex-m3 -mthumb -DMBED_ROM_START=0x8000000 -DMBED_ROM_SIZE=0x10000 -DMBED_RAM_START=0x20000000 -DMBED_RAM_SIZE=0x5000 -DMBED_BOOT_STACK_SIZE=1024 -DXIP_ENABLE=0 
LD_SYS_LIBS :=-Wl,--start-group -lstdc++ -lsupc++ -lm -lc -lgcc -lnosys -lmbed -Wl,--end-group

# Tools and Flags
###############################################################################
# Rules

.PHONY: all lst size


all: $(PROJECT).bin $(PROJECT).hex size


.s.o:
	+@$(call MAKEDIR,$(dir $@))
	+@echo "Assemble: $(notdir $<)"
  
	@$(AS) -c $(ASM_FLAGS) -o $@ $<
  


.S.o:
	+@$(call MAKEDIR,$(dir $@))
	+@echo "Assemble: $(notdir $<)"
  
	@$(AS) -c $(ASM_FLAGS) -o $@ $<
  

.c.o:
	+@$(call MAKEDIR,$(dir $@))
	+@echo "Compile: $(notdir $<)"
	@$(CC) $(C_FLAGS) $(INCLUDE_PATHS) -o $@ $<

.cpp.o:
	+@$(call MAKEDIR,$(dir $@))
	+@echo "Compile: $(notdir $<)"
	@$(CPP) $(CXX_FLAGS) $(INCLUDE_PATHS) -o $@ $<


$(PROJECT).link_script.ld: $(LINKER_SCRIPT)
	@$(PREPROC) $< -o $@


$(PROJECT).elf: $(OBJECTS) $(SYS_OBJECTS) $(PROJECT).link_script.ld 
	$(file > .link_options.txt, $(filter %.o, $^))
	+@echo "link: $(notdir $@)"
	@$(LD) $(LD_FLAGS) -T $(filter-out %.o, $^) $(LIBRARY_PATHS) --output $@ @.link_options.txt $(LIBRARIES) $(LD_SYS_LIBS)


$(PROJECT).bin: $(PROJECT).elf
	$(ELF2BIN) -O binary $< $@
	+@echo "===== bin file ready to flash: $(OBJDIR)/$@ =====" 

$(PROJECT).hex: $(PROJECT).elf
	$(ELF2BIN) -O ihex $< $@
	$(ELF2SIZE) $< $@



# Rules
###############################################################################
# Dependencies

DEPS = $(OBJECTS:.o=.d) $(SYS_OBJECTS:.o=.d)
-include $(DEPS)
endif

# Dependencies
###############################################################################
# Catch-all

%: ;

# Catch-all
###############################################################################
