################################################################################
# MRS Version: 1.9.1
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Task/subdir.mk
-include StdPeriphDriver/subdir.mk
-include Startup/subdir.mk
-include RVMSIS/subdir.mk
-include HAL/subdir.mk
-include APP/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
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

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
M02L_ML307A_app.hexAndbin \

SECONDARY_LIST += \
M02L_ML307A_app.lst \

SECONDARY_SIZE += \
M02L_ML307A_app.siz \


# All Target
all: M02L_ML307A_app.elf secondary-outputs

# Tool invocations
M02L_ML307A_app.elf: $(OBJS) $(USER_OBJS)
	@	@	riscv-none-embed-gcc -march=rv32imac -mabi=ilp32 -mcmodel=medany -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -T "../Ld/Link.ld" -nostartfiles -Xlinker --gc-sections -L"../" -L"R:\CODE\CHcode\CH582\M02L_ML307A\M02L_ML307A_app\LIB" -L"../StdPeriphDriver" -Xlinker --print-memory-usage -Wl,-Map,"M02L_ML307A_app.map" --specs=nano.specs -u _printf_float --specs=nosys.specs -o "M02L_ML307A_app.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@	@
M02L_ML307A_app.hexAndbin: M02L_ML307A_app.elf
	@	riscv-none-embed-objcopy -O ihex "M02L_ML307A_app.elf"  "M02L_ML307A_app.hex" && riscv-none-embed-objcopy -O binary "M02L_ML307A_app.elf"  "M02L_ML307A_app.bin"
	@	@
M02L_ML307A_app.lst: M02L_ML307A_app.elf
	@	riscv-none-embed-objdump --all-headers --demangle --disassemble "M02L_ML307A_app.elf" > "M02L_ML307A_app.lst"
	@	@
M02L_ML307A_app.siz: M02L_ML307A_app.elf
	@	riscv-none-embed-size --format=berkeley "M02L_ML307A_app.elf"
	@	@
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(ASM_UPPER_DEPS)$(OBJS)$(SECONDARY_FLASH)$(SECONDARY_LIST)$(SECONDARY_SIZE)$(ASM_DEPS)$(S_DEPS)$(S_UPPER_DEPS)$(C_DEPS) M02L_ML307A_app.elf
	-@
post-build:
	-riscv-none-embed-objcopy -O ihex "M02L_ML307A_app.elf" "M02L_ML307A_app.hex" && riscv-none-embed-objcopy -O binary "M02L_ML307A_app.elf" "M02L_ML307A_app.bin"
	-@
secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents post-build

-include ../makefile.targets
