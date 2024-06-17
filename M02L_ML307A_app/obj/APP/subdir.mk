################################################################################
# MRS Version: 1.9.1
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../APP/app_central.c \
../APP/app_main.c \
../APP/app_peripheral.c 

OBJS += \
./APP/app_central.o \
./APP/app_main.o \
./APP/app_peripheral.o 

C_DEPS += \
./APP/app_central.d \
./APP/app_main.d \
./APP/app_peripheral.d 


# Each subdirectory must supply rules for building sources it contributes
APP/%.o: ../APP/%.c
	@	@	riscv-none-embed-gcc -march=rv32imac -mabi=ilp32 -mcmodel=medany -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -DDEBUG=0 -DCLK_OSC32K=2 -I"../StdPeriphDriver/inc" -I"R:\CODE\CHcode\CH582\M02L_ML307A\M02L_ML307A_app\Task\inc" -I"R:\CODE\CHcode\CH582\M02L_ML307A\M02L_ML307A_app\APP\include" -I"R:\CODE\CHcode\CH582\M02L_ML307A\M02L_ML307A_app\LIB" -I"R:\CODE\CHcode\CH582\M02L_ML307A\M02L_ML307A_app\HAL\include" -I"R:\CODE\CHcode\CH582\M02L_ML307A\M02L_ML307A_app\APP" -I"../RVMSIS" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

