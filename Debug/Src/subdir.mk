################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/002led_button.c 

OBJS += \
./Src/002led_button.o 

C_DEPS += \
./Src/002led_button.d 


# Each subdirectory must supply rules for building sources it contributes
Src/002led_button.o: ../Src/002led_button.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"C:/Users/OZGUR/Documents/STM32CubeIDE/workspace_1.3.0/stm32f4xx_drivers/driver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/002led_button.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

