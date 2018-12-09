################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include freeRTOS/src/subdir.mk
-include Example/src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: 2°\ Parcial\ wojtyszyn.axf

dependents:
	-cd D:/Facultad/Digitales\ II/Repositorio\ MCU/lpc_chip_175x_6x/Debug && $(MAKE) all

# Tool invocations
2°\ Parcial\ wojtyszyn.axf: $(OBJS) $(USER_OBJS) D:/Facultad/Digitales\ II/Repositorio\ MCU/lpc_chip_175x_6x/Debug/liblpc_chip_175x_6x.a
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"D:\Facultad\Digitales II\Repositorio MCU\lpc_chip_175x_6x\Debug" -Xlinker -Map="2° Parcial wojtyszyn.map" -Xlinker --gc-sections -Xlinker -print-memory-usage -mcpu=cortex-m3 -mthumb -T "FRTOS_Debug.ld" -o "2° Parcial wojtyszyn.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) "2° Parcial wojtyszyn.axf"
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "2° Parcial wojtyszyn.axf"; # arm-none-eabi-objcopy -v -O binary "2° Parcial wojtyszyn.axf" "2° Parcial wojtyszyn.bin" ; # checksum -p LPC1769 -d "2° Parcial wojtyszyn.bin";
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build
D:/Facultad/Digitales\ II/Repositorio\ MCU/lpc_chip_175x_6x/Debug/liblpc_chip_175x_6x.a:

-include ../makefile.targets
