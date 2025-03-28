################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/can.c \
../src/exposure.c \
../src/i2c.c \
../src/main.c \
../src/stepmotor.c \
../src/stm32f4xx_it.c \
../src/syscalls.c \
../src/system_stm32f4xx.c \
../src/uartpc.c 

OBJS += \
./src/can.o \
./src/exposure.o \
./src/i2c.o \
./src/main.o \
./src/stepmotor.o \
./src/stm32f4xx_it.o \
./src/syscalls.o \
./src/system_stm32f4xx.o \
./src/uartpc.o 

C_DEPS += \
./src/can.d \
./src/exposure.d \
./src/i2c.d \
./src/main.d \
./src/stepmotor.d \
./src/stm32f4xx_it.d \
./src/syscalls.d \
./src/system_stm32f4xx.d \
./src/uartpc.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32 -DSTM32F4 -DSTM32F407VGTx -DSTM32F407G_DISC1 -DDEBUG -D#ARM_MATH_CM4 -D#STM32F40_41xxx -D#STM32F40XX -D#USE_STDPERIPH_DRIVER -I"C:/Users/hqasemi/workspace/blinkx/inc" -I"C:/Users/hqasemi/workspace/blinkx/CMSIS/core" -I"C:/Users/hqasemi/workspace/blinkx/CMSIS/device" -I"C:/Users/hqasemi/workspace/blinkx/StdPeriph_Driver/inc" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


