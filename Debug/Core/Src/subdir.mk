################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/lwrb.c \
../Core/Src/main.c \
../Core/Src/mcp23017.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/vl53l0x_class.c 

OBJS += \
./Core/Src/lwrb.o \
./Core/Src/main.o \
./Core/Src/mcp23017.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/vl53l0x_class.o 

C_DEPS += \
./Core/Src/lwrb.d \
./Core/Src/main.d \
./Core/Src/mcp23017.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/vl53l0x_class.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_HAL_DRIVER -DSTM32F411xE -I"/home/coleman/Documents/licc_firm/uart_test_LICC_1_usb_uart/Core/Inc" -I"/home/coleman/Documents/licc_firm/uart_test_LICC_1_usb_uart/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/home/coleman/Documents/licc_firm/uart_test_LICC_1_usb_uart/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"/home/coleman/Documents/licc_firm/uart_test_LICC_1_usb_uart/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/coleman/Documents/licc_firm/uart_test_LICC_1_usb_uart/Drivers/CMSIS/Include"  -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


