################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../startup/startup_stm32.s 

C_SRCS += \
../startup/sysmem.c 

OBJS += \
./startup/startup_stm32.o \
./startup/sysmem.o 

C_DEPS += \
./startup/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
startup/%.o: ../startup/%.s
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Assembler'
	@echo $(PWD)
	arm-none-eabi-as -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -I"C:/Users/hqasemi/workspace/blinkx/StdPeriph_Driver/inc" -I"C:/Users/hqasemi/workspace/blinkx/CMSIS/core" -I"C:/Users/hqasemi/workspace/blinkx/CMSIS/device" -I"C:/Users/hqasemi/workspace/blinkx/inc" -I"C:/Users/hqasemi/workspace/blinkx/StdPeriph_Driver/src" -g -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

startup/%.o: ../startup/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32 -DSTM32F4 -DSTM32F407VGTx -DSTM32F407G_DISC1 -DDEBUG -D#ARM_MATH_CM4 -D#STM32F40_41xxx -D#STM32F40XX -D#USE_STDPERIPH_DRIVER -I"C:/Users/hqasemi/workspace/blinkx/inc" -I"C:/Users/hqasemi/workspace/blinkx/CMSIS/core" -I"C:/Users/hqasemi/workspace/blinkx/CMSIS/device" -I"C:/Users/hqasemi/workspace/blinkx/StdPeriph_Driver/inc" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


