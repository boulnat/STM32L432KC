################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/MCP9600/MCP9600.c 

OBJS += \
./Drivers/MCP9600/MCP9600.o 

C_DEPS += \
./Drivers/MCP9600/MCP9600.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/MCP9600/MCP9600.o: ../Drivers/MCP9600/MCP9600.c Drivers/MCP9600/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/sajl9/OneDrive/Documents/GitHub/STM32L432KC/MB-synthetiseur/Drivers/CanOPEN" -I"C:/Users/sajl9/OneDrive/Documents/GitHub/STM32L432KC/MB-synthetiseur/Drivers/AS7341" -I"C:/Users/sajl9/OneDrive/Documents/GitHub/STM32L432KC/MB-synthetiseur/Drivers/PCA9685" -I"C:/Users/sajl9/OneDrive/Documents/GitHub/STM32L432KC/MB-synthetiseur/Drivers/MCP9600" -I"C:/Users/sajl9/OneDrive/Documents/GitHub/STM32L432KC/MB-synthetiseur/Drivers/INA226" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/MCP9600/MCP9600.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

