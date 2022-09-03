################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../trace/trcKernelPort.c \
../trace/trcSnapshotRecorder.c \
../trace/trcStreamingRecorder.c 

OBJS += \
./trace/trcKernelPort.o \
./trace/trcSnapshotRecorder.o \
./trace/trcStreamingRecorder.o 

C_DEPS += \
./trace/trcKernelPort.d \
./trace/trcSnapshotRecorder.d \
./trace/trcStreamingRecorder.d 


# Each subdirectory must supply rules for building sources it contributes
trace/%.o: ../trace/%.c trace/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L152xE -DDEBUG -c -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-MODBUS_PID/Projects/GCC/trace/include" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-MODBUS_PID/Projects/GCC/trace/config" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-MODBUS_PID/ST_Code/Drivers/CMSIS/Include" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-MODBUS_PID/ST_Code/Drivers/STM32L1xx_HAL_Driver/Inc" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-MODBUS_PID/ST_Code/Drivers/STM32L1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-MODBUS_PID/ST_Code/Drivers/CMSIS/Device/ST/STM32L1xx/Include" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-MODBUS_PID/Config" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Source/include" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Demo/FreeRTOS-MODBUS_PID/ST_Code/Core/Inc" -I"C:/Users/alejo/Documents/containers-project/D-Box/MODBUS-LIB/Inc" -I"C:/Users/alejo/Documents/containers-project/D-Box/FreeRTOS/Source/portable/GCC/ARM_CM3" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-trace

clean-trace:
	-$(RM) ./trace/trcKernelPort.d ./trace/trcKernelPort.o ./trace/trcSnapshotRecorder.d ./trace/trcSnapshotRecorder.o ./trace/trcStreamingRecorder.d ./trace/trcStreamingRecorder.o

.PHONY: clean-trace

