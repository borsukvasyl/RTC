################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/gpio.c \
../Src/lcd5110.c \
../Src/lcd5110_hal.c \
../Src/lcd5110_ll.c \
../Src/main.c \
../Src/spi.c \
../Src/stm32f3xx_hal_msp.c \
../Src/stm32f3xx_it.c \
../Src/system_stm32f3xx.c 

OBJS += \
./Src/gpio.o \
./Src/lcd5110.o \
./Src/lcd5110_hal.o \
./Src/lcd5110_ll.o \
./Src/main.o \
./Src/spi.o \
./Src/stm32f3xx_hal_msp.o \
./Src/stm32f3xx_it.o \
./Src/system_stm32f3xx.o 

C_DEPS += \
./Src/gpio.d \
./Src/lcd5110.d \
./Src/lcd5110_hal.d \
./Src/lcd5110_ll.d \
./Src/main.d \
./Src/spi.d \
./Src/stm32f3xx_hal_msp.d \
./Src/stm32f3xx_it.d \
./Src/system_stm32f3xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F303xC -I"/media/ivosar/2AC2E2F12DC00F2B/CS@UCU/POK/workspace/RemoteSystemsTempFiles/Display/Inc" -I"/media/ivosar/2AC2E2F12DC00F2B/CS@UCU/POK/workspace/RemoteSystemsTempFiles/Display/Drivers/STM32F3xx_HAL_Driver/Inc" -I"/media/ivosar/2AC2E2F12DC00F2B/CS@UCU/POK/workspace/RemoteSystemsTempFiles/Display/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"/media/ivosar/2AC2E2F12DC00F2B/CS@UCU/POK/workspace/RemoteSystemsTempFiles/Display/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"/media/ivosar/2AC2E2F12DC00F2B/CS@UCU/POK/workspace/RemoteSystemsTempFiles/Display/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


