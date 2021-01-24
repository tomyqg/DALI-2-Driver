################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F0xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
3342_014_DALIsensor.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
3342_014_DALIsensor.list \

OBJCOPY_HEX += \
3342_014_DALIsensor.hex \

OBJCOPY_BIN += \
3342_014_DALIsensor.bin \

OBJCOPY_SREC += \
3342_014_DALIsensor.srec \


# All Target
all: 3342_014_DALIsensor.elf secondary-outputs

# Tool invocations
3342_014_DALIsensor.elf: $(OBJS) $(USER_OBJS) F:\Misc\3342_014_DALIsensor\STM32F051K8UX_FLASH.ld
	arm-none-eabi-gcc -o "3342_014_DALIsensor.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0 -T"F:\Misc\3342_014_DALIsensor\STM32F051K8UX_FLASH.ld" --specs=nosys.specs -Wl,-Map="3342_014_DALIsensor.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

3342_014_DALIsensor.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "3342_014_DALIsensor.list"
	@echo 'Finished building: $@'
	@echo ' '

3342_014_DALIsensor.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "3342_014_DALIsensor.hex"
	@echo 'Finished building: $@'
	@echo ' '

3342_014_DALIsensor.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "3342_014_DALIsensor.bin"
	@echo 'Finished building: $@'
	@echo ' '

3342_014_DALIsensor.srec: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O srec $(EXECUTABLES) "3342_014_DALIsensor.srec"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN) $(OBJCOPY_SREC)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
