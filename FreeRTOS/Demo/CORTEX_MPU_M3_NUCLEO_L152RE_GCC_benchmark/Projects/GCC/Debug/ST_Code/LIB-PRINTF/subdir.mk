################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/alejo/Documents/containers-project/MCU_containers-code/Development/FreeRTOS/FreeRTOS/Demo/CORTEX_MPU_M3_NUCLEO_L152RE_GCC_benchmark/ST_Code/LIB-PRINTF/printf.c 

OBJS += \
./ST_Code/LIB-PRINTF/printf.o 

C_DEPS += \
./ST_Code/LIB-PRINTF/printf.d 


# Each subdirectory must supply rules for building sources it contributes
ST_Code/LIB-PRINTF/printf.o: C:/Users/alejo/Documents/containers-project/MCU_containers-code/Development/FreeRTOS/FreeRTOS/Demo/CORTEX_MPU_M3_NUCLEO_L152RE_GCC_benchmark/ST_Code/LIB-PRINTF/printf.c ST_Code/LIB-PRINTF/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L152xE -DDEBUG -DUSE_FULL_LL_DRIVER -c -I"C:/Users/alejo/Documents/containers-project/MCU_containers-code/Development/FreeRTOS/FreeRTOS/Demo/CORTEX_MPU_M3_NUCLEO_L152RE_GCC_benchmark/ST_Code/Drivers/CMSIS/Include" -I"C:/Users/alejo/Documents/containers-project/MCU_containers-code/Development/FreeRTOS/FreeRTOS/Demo/CORTEX_MPU_M3_NUCLEO_L152RE_GCC_benchmark/ST_Code/LIB-PRINTF" -I"C:/Users/alejo/Documents/containers-project/MCU_containers-code/Development/FreeRTOS/FreeRTOS/Demo/CORTEX_MPU_M3_NUCLEO_L152RE_GCC_benchmark/ST_Code/Drivers/STM32L1xx_HAL_Driver/Inc" -I"C:/Users/alejo/Documents/containers-project/MCU_containers-code/Development/FreeRTOS/FreeRTOS/Demo/CORTEX_MPU_M3_NUCLEO_L152RE_GCC_benchmark/ST_Code/Drivers/STM32L1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/alejo/Documents/containers-project/MCU_containers-code/Development/FreeRTOS/FreeRTOS/Demo/CORTEX_MPU_M3_NUCLEO_L152RE_GCC_benchmark/ST_Code/Drivers/CMSIS/Device/ST/STM32L1xx/Include" -I"C:/Users/alejo/Documents/containers-project/MCU_containers-code/Development/FreeRTOS/FreeRTOS/Demo/CORTEX_MPU_M3_NUCLEO_L152RE_GCC_benchmark/Config" -I"C:/Users/alejo/Documents/containers-project/MCU_containers-code/Development/FreeRTOS/FreeRTOS/Demo/CORTEX_MPU_M3_NUCLEO_L152RE_GCC_benchmark/Demo" -I"C:/Users/alejo/Documents/containers-project/MCU_containers-code/Development/FreeRTOS/FreeRTOS/Source/include" -I"C:/Users/alejo/Documents/containers-project/MCU_containers-code/Development/FreeRTOS/FreeRTOS/Demo/CORTEX_MPU_M3_NUCLEO_L152RE_GCC_benchmark/ST_Code/trace/include" -I"C:/Users/alejo/Documents/containers-project/MCU_containers-code/Development/FreeRTOS/FreeRTOS/Demo/CORTEX_MPU_M3_NUCLEO_L152RE_GCC_benchmark/ST_Code/trace/config" -I"C:/Users/alejo/Documents/containers-project/MCU_containers-code/Development/FreeRTOS/FreeRTOS/Source/portable/GCC/ARM_CM3_MPU" -I"C:/Users/alejo/Documents/containers-project/MCU_containers-code/Development/FreeRTOS/FreeRTOS/Demo/CORTEX_MPU_M3_NUCLEO_L152RE_GCC_benchmark/ST_Code/Core/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-ST_Code-2f-LIB-2d-PRINTF

clean-ST_Code-2f-LIB-2d-PRINTF:
	-$(RM) ./ST_Code/LIB-PRINTF/printf.d ./ST_Code/LIB-PRINTF/printf.o

.PHONY: clean-ST_Code-2f-LIB-2d-PRINTF

