################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/PCA9685/PCA9685.c 

OBJS += \
./Drivers/PCA9685/PCA9685.o 

C_DEPS += \
./Drivers/PCA9685/PCA9685.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/PCA9685/PCA9685.o: ../Drivers/PCA9685/PCA9685.c Drivers/PCA9685/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I../Drivers/AS7341 -I../Drivers/INA226 -I../Drivers/MCP9600 -I../Drivers/PCA9685 -I../Drivers/PID -I../Drivers/CanOPEN -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/PCA9685/PCA9685.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

