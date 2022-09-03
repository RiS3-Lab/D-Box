################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include ST_Code/trace/subdir.mk
-include ST_Code/LIB-PRINTF/subdir.mk
-include ST_Code/Drivers/STM32L1xx_HAL_Driver/Src/subdir.mk
-include ST_Code/Core/Src/subdir.mk
-include FreeRTOS/portable/MemMang/subdir.mk
-include FreeRTOS/portable/GCC/ARM_CM3_MPU/subdir.mk
-include FreeRTOS/portable/Common/subdir.mk
-include FreeRTOS/subdir.mk
-include Demo/subdir.mk
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


BUILD_ARTIFACT_NAME := FreeRTOSDemoBench
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
FreeRTOSDemoBench.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
FreeRTOSDemoBench.list \

OBJCOPY_BIN += \
FreeRTOSDemoBench.bin \


# All Target
all: main-build

# Main-build Target
main-build: FreeRTOSDemoBench.elf secondary-outputs

# Tool invocations
FreeRTOSDemoBench.elf: $(OBJS) $(USER_OBJS) C:\Users\alejo\Documents\containers-project\MCU_containers-code\Development\FreeRTOS\FreeRTOS\Demo\CORTEX_MPU_M3_NUCLEO_L152RE_GCC_benchmark\Projects\GCC\STM32L152RETX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "FreeRTOSDemoBench.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\alejo\Documents\containers-project\MCU_containers-code\Development\FreeRTOS\FreeRTOS\Demo\CORTEX_MPU_M3_NUCLEO_L152RE_GCC_benchmark\Projects\GCC\STM32L152RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="FreeRTOSDemoBench.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

FreeRTOSDemoBench.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "FreeRTOSDemoBench.list"
	@echo 'Finished building: $@'
	@echo ' '

FreeRTOSDemoBench.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "FreeRTOSDemoBench.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) FreeRTOSDemoBench.bin FreeRTOSDemoBench.elf FreeRTOSDemoBench.list default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
