################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../driver/Src/stm32f407xx_gpio_driver.c 

OBJS += \
./driver/Src/stm32f407xx_gpio_driver.o 

C_DEPS += \
./driver/Src/stm32f407xx_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
driver/Src/stm32f407xx_gpio_driver.o: ../driver/Src/stm32f407xx_gpio_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"C:/Users/OZGUR/Documents/STM32CubeIDE/workspace_1.3.0/stm32f4xx_drivers/driver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"driver/Src/stm32f407xx_gpio_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

