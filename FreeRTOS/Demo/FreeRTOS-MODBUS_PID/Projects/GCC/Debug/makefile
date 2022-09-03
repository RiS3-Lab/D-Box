################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include trace/subdir.mk
-include Startup/subdir.mk
-include ST_Code/Drivers/STM32L1xx_HAL_Driver/Src/subdir.mk
-include ST_Code/Core/Src/subdir.mk
-include MODBUS-LIB/Src/subdir.mk
-include FreeRTOS/portable/MemMang/subdir.mk
-include FreeRTOS/portable/GCC/ARM_CM3/subdir.mk
-include FreeRTOS/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := L152RE-FreeRTOS-MODBUS
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
L152RE-FreeRTOS-MODBUS.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
L152RE-FreeRTOS-MODBUS.list \

OBJCOPY_BIN += \
L152RE-FreeRTOS-MODBUS.bin \


# All Target
all: main-build

# Main-build Target
main-build: L152RE-FreeRTOS-MODBUS.elf secondary-outputs

# Tool invocations
L152RE-FreeRTOS-MODBUS.elf: $(OBJS) $(USER_OBJS) C:\Users\alejo\Documents\containers-project\D-Box\FreeRTOS\Demo\FreeRTOS-MODBUS_PID\Projects\GCC\STM32L152RETX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "L152RE-FreeRTOS-MODBUS.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\alejo\Documents\containers-project\D-Box\FreeRTOS\Demo\FreeRTOS-MODBUS_PID\Projects\GCC\STM32L152RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="L152RE-FreeRTOS-MODBUS.map" -Wl,--gc-sections -static -L"C:\Users\alejo\Documents\containers-project\D-Box\FreeRTOS\Demo\FreeRTOS-MODBUS_PID\Projects\GCC\lib" --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

L152RE-FreeRTOS-MODBUS.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "L152RE-FreeRTOS-MODBUS.list"
	@echo 'Finished building: $@'
	@echo ' '

L152RE-FreeRTOS-MODBUS.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "L152RE-FreeRTOS-MODBUS.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) L152RE-FreeRTOS-MODBUS.bin L152RE-FreeRTOS-MODBUS.elf L152RE-FreeRTOS-MODBUS.list default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
