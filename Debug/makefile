################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32L0xx_HAL_Driver/Src/subdir.mk
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
RHF76-052.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
RHF76-052.list \

OBJCOPY_BIN += \
RHF76-052.bin \


# All Target
all: RHF76-052.elf secondary-outputs

# Tool invocations
RHF76-052.elf: $(OBJS) $(USER_OBJS) E:\Biblio\Documents\STM32CubeIDE\workspace_1.3.0\RHF76-052\STM32L051C8TX_FLASH.ld
	arm-none-eabi-gcc -o "RHF76-052.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0plus -T"E:\Biblio\Documents\STM32CubeIDE\workspace_1.3.0\RHF76-052\STM32L051C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="RHF76-052.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

RHF76-052.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "RHF76-052.list"
	@echo 'Finished building: $@'
	@echo ' '

RHF76-052.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "RHF76-052.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
