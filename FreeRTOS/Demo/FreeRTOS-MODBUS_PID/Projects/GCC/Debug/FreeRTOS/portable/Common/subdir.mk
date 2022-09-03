################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Source/portable/Common/mpu_wrappers.c 

OBJS += \
./FreeRTOS/portable/Common/mpu_wrappers.o 

C_DEPS += \
./FreeRTOS/portable/Common/mpu_wrappers.d 


# Each subdirectory must supply rules for building sources it contributes
FreeRTOS/portable/Common/mpu_wrappers.o: C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Source/portable/Common/mpu_wrappers.c FreeRTOS/portable/Common/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L152xE -DDEBUG -c -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-MODBUS_PID/Projects/GCC/trace/include" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-MODBUS_PID/Projects/GCC/trace/config" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-MODBUS_PID/ST_Code/Drivers/CMSIS/Include" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-MODBUS_PID/ST_Code/Drivers/STM32L1xx_HAL_Driver/Inc" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-MODBUS_PID/ST_Code/Drivers/STM32L1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-MODBUS_PID/ST_Code/Drivers/CMSIS/Device/ST/STM32L1xx/Include" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-MODBUS_PID/Config" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Source/include" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-MODBUS_PID/ST_Code/Core/Inc" -I"C:/Users/alejo/Documents/containers-project/D-Box/MODBUS-LIB/Inc" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Source/portable/GCC/ARM_CM3" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-FreeRTOS-2f-portable-2f-Common

clean-FreeRTOS-2f-portable-2f-Common:
	-$(RM) ./FreeRTOS/portable/Common/mpu_wrappers.d ./FreeRTOS/portable/Common/mpu_wrappers.o

.PHONY: clean-FreeRTOS-2f-portable-2f-Common

