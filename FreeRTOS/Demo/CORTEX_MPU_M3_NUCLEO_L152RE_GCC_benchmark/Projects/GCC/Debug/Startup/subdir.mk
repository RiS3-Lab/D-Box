################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Startup/startup_stm32l152retx.s 

C_SRCS += \
../Startup/memfault_handler.c \
../Startup/syscalls.c \
../Startup/sysmem.c 

OBJS += \
./Startup/memfault_handler.o \
./Startup/startup_stm32l152retx.o \
./Startup/syscalls.o \
./Startup/sysmem.o 

S_DEPS += \
./Startup/startup_stm32l152retx.d 

C_DEPS += \
./Startup/memfault_handler.d \
./Startup/syscalls.d \
./Startup/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Startup/%.o: ../Startup/%.c Startup/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L152xE -DDEBUG -DUSE_FULL_LL_DRIVER -c -I"C:/Users/alejo/Documents/containers-project/MCU_containers-code/Development/FreeRTOS/FreeRTOS/Demo/CORTEX_MPU_M3_NUCLEO_L152RE_GCC_benchmark/ST_Code/Drivers/CMSIS/Include" -I"C:/Users/alejo/Documents/containers-project/MCU_containers-code/Development/FreeRTOS/FreeRTOS/Demo/CORTEX_MPU_M3_NUCLEO_L152RE_GCC_benchmark/ST_Code/LIB-PRINTF" -I"C:/Users/alejo/Documents/containers-project/MCU_containers-code/Development/FreeRTOS/FreeRTOS/Demo/CORTEX_MPU_M3_NUCLEO_L152RE_GCC_benchmark/ST_Code/Drivers/STM32L1xx_HAL_Driver/Inc" -I"C:/Users/alejo/Documents/containers-project/MCU_containers-code/Development/FreeRTOS/FreeRTOS/Demo/CORTEX_MPU_M3_NUCLEO_L152RE_GCC_benchmark/ST_Code/Drivers/STM32L1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/alejo/Documents/containers-project/MCU_containers-code/Development/FreeRTOS/FreeRTOS/Demo/CORTEX_MPU_M3_NUCLEO_L152RE_GCC_benchmark/ST_Code/Drivers/CMSIS/Device/ST/STM32L1xx/Include" -I"C:/Users/alejo/Documents/containers-project/MCU_containers-code/Development/FreeRTOS/FreeRTOS/Demo/CORTEX_MPU_M3_NUCLEO_L152RE_GCC_benchmark/Config" -I"C:/Users/alejo/Documents/containers-project/MCU_containers-code/Development/FreeRTOS/FreeRTOS/Demo/CORTEX_MPU_M3_NUCLEO_L152RE_GCC_benchmark/Demo" -I"C:/Users/alejo/Documents/containers-project/MCU_containers-code/Development/FreeRTOS/FreeRTOS/Source/include" -I"C:/Users/alejo/Documents/containers-project/MCU_containers-code/Development/FreeRTOS/FreeRTOS/Demo/CORTEX_MPU_M3_NUCLEO_L152RE_GCC_benchmark/ST_Code/trace/include" -I"C:/Users/alejo/Documents/containers-project/MCU_containers-code/Development/FreeRTOS/FreeRTOS/Demo/CORTEX_MPU_M3_NUCLEO_L152RE_GCC_benchmark/ST_Code/trace/config" -I"C:/Users/alejo/Documents/containers-project/MCU_containers-code/Development/FreeRTOS/FreeRTOS/Source/portable/GCC/ARM_CM3_MPU" -I"C:/Users/alejo/Documents/containers-project/MCU_containers-code/Development/FreeRTOS/FreeRTOS/Demo/CORTEX_MPU_M3_NUCLEO_L152RE_GCC_benchmark/ST_Code/Core/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Startup/%.o: ../Startup/%.s Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m3 -g3 -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

clean: clean-Startup

clean-Startup:
	-$(RM) ./Startup/memfault_handler.d ./Startup/memfault_handler.o ./Startup/startup_stm32l152retx.d ./Startup/startup_stm32l152retx.o ./Startup/syscalls.d ./Startup/syscalls.o ./Startup/sysmem.d ./Startup/sysmem.o

.PHONY: clean-Startup

