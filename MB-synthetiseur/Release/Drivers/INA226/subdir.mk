################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/INA226/INA226.c 

OBJS += \
./Drivers/INA226/INA226.o 

C_DEPS += \
./Drivers/INA226/INA226.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/INA226/INA226.o: ../Drivers/INA226/INA226.c Drivers/INA226/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Drivers/CanOPEN -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/CanOPEN" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/PCA9685" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/MCP9600" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/INA226" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/STM32L4xx_HAL_Driver" -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/STM32L4xx_HAL_Driver/Inc" -I../Drivers/CMSIS/Device/ST/STM32L4xx/Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/INA226/INA226.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

