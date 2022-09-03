################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/alejo/Documents/containers-project/D-Box/TRACE-LIB/trcKernelPort.c \
C:/Users/alejo/Documents/containers-project/D-Box/TRACE-LIB/trcSnapshotRecorder.c \
C:/Users/alejo/Documents/containers-project/D-Box/TRACE-LIB/trcStreamingRecorder.c 

OBJS += \
./TRACE-LIB/trcKernelPort.o \
./TRACE-LIB/trcSnapshotRecorder.o \
./TRACE-LIB/trcStreamingRecorder.o 

C_DEPS += \
./TRACE-LIB/trcKernelPort.d \
./TRACE-LIB/trcSnapshotRecorder.d \
./TRACE-LIB/trcStreamingRecorder.d 


# Each subdirectory must supply rules for building sources it contributes
TRACE-LIB/trcKernelPort.o: C:/Users/alejo/Documents/containers-project/D-Box/TRACE-LIB/trcKernelPort.c TRACE-LIB/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L152xE -DDEBUG -DUSE_FULL_LL_DRIVER -c -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Drivers/CMSIS/Include" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Drivers/STM32L1xx_HAL_Driver/Inc" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Drivers/STM32L1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Drivers/CMSIS/Device/ST/STM32L1xx/Include" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/Config" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Source/include" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Source/portable/GCC/ARM_CM3_MPU" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Core/Inc" -I"C:/Users/alejo/Documents/containers-project/D-Box/MODBUS-LIB/Inc" -I"C:/Users/alejo/Documents/containers-project/D-Box/TRACE-LIB/config" -I"C:/Users/alejo/Documents/containers-project/D-Box/TRACE-LIB/include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
TRACE-LIB/trcSnapshotRecorder.o: C:/Users/alejo/Documents/containers-project/D-Box/TRACE-LIB/trcSnapshotRecorder.c TRACE-LIB/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L152xE -DDEBUG -DUSE_FULL_LL_DRIVER -c -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Drivers/CMSIS/Include" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Drivers/STM32L1xx_HAL_Driver/Inc" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Drivers/STM32L1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Drivers/CMSIS/Device/ST/STM32L1xx/Include" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/Config" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Source/include" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Source/portable/GCC/ARM_CM3_MPU" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Core/Inc" -I"C:/Users/alejo/Documents/containers-project/D-Box/MODBUS-LIB/Inc" -I"C:/Users/alejo/Documents/containers-project/D-Box/TRACE-LIB/config" -I"C:/Users/alejo/Documents/containers-project/D-Box/TRACE-LIB/include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
TRACE-LIB/trcStreamingRecorder.o: C:/Users/alejo/Documents/containers-project/D-Box/TRACE-LIB/trcStreamingRecorder.c TRACE-LIB/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L152xE -DDEBUG -DUSE_FULL_LL_DRIVER -c -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Drivers/CMSIS/Include" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Drivers/STM32L1xx_HAL_Driver/Inc" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Drivers/STM32L1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Drivers/CMSIS/Device/ST/STM32L1xx/Include" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/Config" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Source/include" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Source/portable/GCC/ARM_CM3_MPU" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-BOX-MODBUS_PID/ST_Code/Core/Inc" -I"C:/Users/alejo/Documents/containers-project/D-Box/MODBUS-LIB/Inc" -I"C:/Users/alejo/Documents/containers-project/D-Box/TRACE-LIB/config" -I"C:/Users/alejo/Documents/containers-project/D-Box/TRACE-LIB/include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-TRACE-2d-LIB

clean-TRACE-2d-LIB:
	-$(RM) ./TRACE-LIB/trcKernelPort.d ./TRACE-LIB/trcKernelPort.o ./TRACE-LIB/trcSnapshotRecorder.d ./TRACE-LIB/trcSnapshotRecorder.o ./TRACE-LIB/trcStreamingRecorder.d ./TRACE-LIB/trcStreamingRecorder.o

.PHONY: clean-TRACE-2d-LIB

