################################################################################
# MRS Version: 1.9.1
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../RVMSIS/core_riscv.c 

OBJS += \
./RVMSIS/core_riscv.o 

C_DEPS += \
./RVMSIS/core_riscv.d 


# Each subdirectory must supply rules for building sources it contributes
RVMSIS/%.o: ../RVMSIS/%.c
	@	@	riscv-none-embed-gcc -march=rv32imac -mabi=ilp32 -mcmodel=medany -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -DDEBUG=0 -DCLK_OSC32K=2 -I"../StdPeriphDriver/inc" -I"R:\CODE\CHcode\CH582\M02L_ML307A\M02L_ML307A_app\Task\inc" -I"R:\CODE\CHcode\CH582\M02L_ML307A\M02L_ML307A_app\APP\include" -I"R:\CODE\CHcode\CH582\M02L_ML307A\M02L_ML307A_app\LIB" -I"R:\CODE\CHcode\CH582\M02L_ML307A\M02L_ML307A_app\HAL\include" -I"R:\CODE\CHcode\CH582\M02L_ML307A\M02L_ML307A_app\APP" -I"../RVMSIS" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

