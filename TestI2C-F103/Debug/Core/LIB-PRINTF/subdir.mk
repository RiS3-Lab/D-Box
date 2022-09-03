################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/LIB-PRINTF/printf.c 

OBJS += \
./Core/LIB-PRINTF/printf.o 

C_DEPS += \
./Core/LIB-PRINTF/printf.d 


# Each subdirectory must supply rules for building sources it contributes
Core/LIB-PRINTF/%.o: ../Core/LIB-PRINTF/%.c Core/LIB-PRINTF/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/alejo/Documents/containers-project/MCU_containers-code/MCU_containers-writeups/evaluation/performance/TestI2C-F103/Core/LIB-PRINTF" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-LIB-2d-PRINTF

clean-Core-2f-LIB-2d-PRINTF:
	-$(RM) ./Core/LIB-PRINTF/printf.d ./Core/LIB-PRINTF/printf.o

.PHONY: clean-Core-2f-LIB-2d-PRINTF

