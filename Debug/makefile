################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
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
MUSCLEmaster.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
MUSCLEmaster.list \


# All Target
all: MUSCLEmaster.elf secondary-outputs

# Tool invocations
MUSCLEmaster.elf: $(OBJS) $(USER_OBJS) /Users/kohstall/STM32CubeIDE/workspace_1.0.2/MUSCLEmaster/STM32F405VGTX_FLASH.ld
	arm-none-eabi-gcc -o "MUSCLEmaster.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/Users/kohstall/STM32CubeIDE/workspace_1.0.2/MUSCLEmaster/STM32F405VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="MUSCLEmaster.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

MUSCLEmaster.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "MUSCLEmaster.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
