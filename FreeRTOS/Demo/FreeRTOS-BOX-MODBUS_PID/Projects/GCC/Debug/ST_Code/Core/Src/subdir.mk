################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Core/Src/main.c \
C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Core/Src/stm32l1xx_hal_timebase_tim.c \
C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Core/Src/stm32l1xx_it.c \
C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Core/Src/system_stm32l1xx.c 

OBJS += \
./ST_Code/Core/Src/main.o \
./ST_Code/Core/Src/stm32l1xx_hal_timebase_tim.o \
./ST_Code/Core/Src/stm32l1xx_it.o \
./ST_Code/Core/Src/system_stm32l1xx.o 

C_DEPS += \
./ST_Code/Core/Src/main.d \
./ST_Code/Core/Src/stm32l1xx_hal_timebase_tim.d \
./ST_Code/Core/Src/stm32l1xx_it.d \
./ST_Code/Core/Src/system_stm32l1xx.d 


# Each subdirectory must supply rules for building sources it contributes
ST_Code/Core/Src/main.o: C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Core/Src/main.c ST_Code/Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L152xE -DDEBUG -DUSE_FULL_LL_DRIVER -c -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Drivers/CMSIS/Include" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Drivers/STM32L1xx_HAL_Driver/Inc" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Drivers/STM32L1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Drivers/CMSIS/Device/ST/STM32L1xx/Include" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/Config" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Source/include" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Source/portable/GCC/ARM_CM3_MPU" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Core/Inc" -I"C:/Users/alejo/Documents/containers-project/D-Box/MODBUS-LIB/Inc" -I"C:/Users/alejo/Documents/containers-project/D-Box/TRACE-LIB/config" -I"C:/Users/alejo/Documents/containers-project/D-Box/TRACE-LIB/include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
ST_Code/Core/Src/stm32l1xx_hal_timebase_tim.o: C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Core/Src/stm32l1xx_hal_timebase_tim.c ST_Code/Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L152xE -DDEBUG -DUSE_FULL_LL_DRIVER -c -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Drivers/CMSIS/Include" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Drivers/STM32L1xx_HAL_Driver/Inc" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Drivers/STM32L1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Drivers/CMSIS/Device/ST/STM32L1xx/Include" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/Config" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Source/include" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Source/portable/GCC/ARM_CM3_MPU" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Core/Inc" -I"C:/Users/alejo/Documents/containers-project/D-Box/MODBUS-LIB/Inc" -I"C:/Users/alejo/Documents/containers-project/D-Box/TRACE-LIB/config" -I"C:/Users/alejo/Documents/containers-project/D-Box/TRACE-LIB/include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
ST_Code/Core/Src/stm32l1xx_it.o: C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Core/Src/stm32l1xx_it.c ST_Code/Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L152xE -DDEBUG -DUSE_FULL_LL_DRIVER -c -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Drivers/CMSIS/Include" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Drivers/STM32L1xx_HAL_Driver/Inc" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Drivers/STM32L1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Drivers/CMSIS/Device/ST/STM32L1xx/Include" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/Config" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Source/include" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Source/portable/GCC/ARM_CM3_MPU" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Core/Inc" -I"C:/Users/alejo/Documents/containers-project/D-Box/MODBUS-LIB/Inc" -I"C:/Users/alejo/Documents/containers-project/D-Box/TRACE-LIB/config" -I"C:/Users/alejo/Documents/containers-project/D-Box/TRACE-LIB/include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
ST_Code/Core/Src/system_stm32l1xx.o: C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Core/Src/system_stm32l1xx.c ST_Code/Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L152xE -DDEBUG -DUSE_FULL_LL_DRIVER -c -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Drivers/CMSIS/Include" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Drivers/STM32L1xx_HAL_Driver/Inc" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Drivers/STM32L1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Drivers/CMSIS/Device/ST/STM32L1xx/Include" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/Config" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Source/include" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Source/portable/GCC/ARM_CM3_MPU" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Core/Inc" -I"C:/Users/alejo/Documents/containers-project/D-Box/MODBUS-LIB/Inc" -I"C:/Users/alejo/Documents/containers-project/D-Box/TRACE-LIB/config" -I"C:/Users/alejo/Documents/containers-project/D-Box/TRACE-LIB/include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-ST_Code-2f-Core-2f-Src

clean-ST_Code-2f-Core-2f-Src:
	-$(RM) ./ST_Code/Core/Src/main.d ./ST_Code/Core/Src/main.o ./ST_Code/Core/Src/stm32l1xx_hal_timebase_tim.d ./ST_Code/Core/Src/stm32l1xx_hal_timebase_tim.o ./ST_Code/Core/Src/stm32l1xx_it.d ./ST_Code/Core/Src/stm32l1xx_it.o ./ST_Code/Core/Src/system_stm32l1xx.d ./ST_Code/Core/Src/system_stm32l1xx.o

.PHONY: clean-ST_Code-2f-Core-2f-Src

