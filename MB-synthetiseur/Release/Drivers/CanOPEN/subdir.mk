################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CanOPEN/CO_Emergency.c \
../Drivers/CanOPEN/CO_HBconsumer.c \
../Drivers/CanOPEN/CO_NMT_Heartbeat.c \
../Drivers/CanOPEN/CO_PDO.c \
../Drivers/CanOPEN/CO_SDO.c \
../Drivers/CanOPEN/CO_SDOmaster.c \
../Drivers/CanOPEN/CO_SYNC.c \
../Drivers/CanOPEN/CO_driver.c \
../Drivers/CanOPEN/CO_trace.c \
../Drivers/CanOPEN/crc16-ccitt.c 

OBJS += \
./Drivers/CanOPEN/CO_Emergency.o \
./Drivers/CanOPEN/CO_HBconsumer.o \
./Drivers/CanOPEN/CO_NMT_Heartbeat.o \
./Drivers/CanOPEN/CO_PDO.o \
./Drivers/CanOPEN/CO_SDO.o \
./Drivers/CanOPEN/CO_SDOmaster.o \
./Drivers/CanOPEN/CO_SYNC.o \
./Drivers/CanOPEN/CO_driver.o \
./Drivers/CanOPEN/CO_trace.o \
./Drivers/CanOPEN/crc16-ccitt.o 

C_DEPS += \
./Drivers/CanOPEN/CO_Emergency.d \
./Drivers/CanOPEN/CO_HBconsumer.d \
./Drivers/CanOPEN/CO_NMT_Heartbeat.d \
./Drivers/CanOPEN/CO_PDO.d \
./Drivers/CanOPEN/CO_SDO.d \
./Drivers/CanOPEN/CO_SDOmaster.d \
./Drivers/CanOPEN/CO_SYNC.d \
./Drivers/CanOPEN/CO_driver.d \
./Drivers/CanOPEN/CO_trace.d \
./Drivers/CanOPEN/crc16-ccitt.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CanOPEN/CO_Emergency.o: ../Drivers/CanOPEN/CO_Emergency.c Drivers/CanOPEN/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Drivers/CanOPEN -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/CanOPEN" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/PCA9685" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/MCP9600" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/INA226" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/STM32L4xx_HAL_Driver" -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/STM32L4xx_HAL_Driver/Inc" -I../Drivers/CMSIS/Device/ST/STM32L4xx/Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/CanOPEN/CO_Emergency.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CanOPEN/CO_HBconsumer.o: ../Drivers/CanOPEN/CO_HBconsumer.c Drivers/CanOPEN/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Drivers/CanOPEN -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/CanOPEN" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/PCA9685" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/MCP9600" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/INA226" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/STM32L4xx_HAL_Driver" -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/STM32L4xx_HAL_Driver/Inc" -I../Drivers/CMSIS/Device/ST/STM32L4xx/Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/CanOPEN/CO_HBconsumer.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CanOPEN/CO_NMT_Heartbeat.o: ../Drivers/CanOPEN/CO_NMT_Heartbeat.c Drivers/CanOPEN/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Drivers/CanOPEN -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/CanOPEN" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/PCA9685" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/MCP9600" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/INA226" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/STM32L4xx_HAL_Driver" -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/STM32L4xx_HAL_Driver/Inc" -I../Drivers/CMSIS/Device/ST/STM32L4xx/Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/CanOPEN/CO_NMT_Heartbeat.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CanOPEN/CO_PDO.o: ../Drivers/CanOPEN/CO_PDO.c Drivers/CanOPEN/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Drivers/CanOPEN -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/CanOPEN" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/PCA9685" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/MCP9600" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/INA226" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/STM32L4xx_HAL_Driver" -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/STM32L4xx_HAL_Driver/Inc" -I../Drivers/CMSIS/Device/ST/STM32L4xx/Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/CanOPEN/CO_PDO.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CanOPEN/CO_SDO.o: ../Drivers/CanOPEN/CO_SDO.c Drivers/CanOPEN/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Drivers/CanOPEN -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/CanOPEN" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/PCA9685" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/MCP9600" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/INA226" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/STM32L4xx_HAL_Driver" -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/STM32L4xx_HAL_Driver/Inc" -I../Drivers/CMSIS/Device/ST/STM32L4xx/Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/CanOPEN/CO_SDO.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CanOPEN/CO_SDOmaster.o: ../Drivers/CanOPEN/CO_SDOmaster.c Drivers/CanOPEN/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Drivers/CanOPEN -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/CanOPEN" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/PCA9685" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/MCP9600" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/INA226" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/STM32L4xx_HAL_Driver" -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/STM32L4xx_HAL_Driver/Inc" -I../Drivers/CMSIS/Device/ST/STM32L4xx/Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/CanOPEN/CO_SDOmaster.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CanOPEN/CO_SYNC.o: ../Drivers/CanOPEN/CO_SYNC.c Drivers/CanOPEN/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Drivers/CanOPEN -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/CanOPEN" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/PCA9685" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/MCP9600" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/INA226" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/STM32L4xx_HAL_Driver" -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/STM32L4xx_HAL_Driver/Inc" -I../Drivers/CMSIS/Device/ST/STM32L4xx/Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/CanOPEN/CO_SYNC.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CanOPEN/CO_driver.o: ../Drivers/CanOPEN/CO_driver.c Drivers/CanOPEN/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Drivers/CanOPEN -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/CanOPEN" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/PCA9685" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/MCP9600" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/INA226" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/STM32L4xx_HAL_Driver" -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/STM32L4xx_HAL_Driver/Inc" -I../Drivers/CMSIS/Device/ST/STM32L4xx/Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/CanOPEN/CO_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CanOPEN/CO_trace.o: ../Drivers/CanOPEN/CO_trace.c Drivers/CanOPEN/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Drivers/CanOPEN -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/CanOPEN" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/PCA9685" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/MCP9600" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/INA226" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/STM32L4xx_HAL_Driver" -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/STM32L4xx_HAL_Driver/Inc" -I../Drivers/CMSIS/Device/ST/STM32L4xx/Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/CanOPEN/CO_trace.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CanOPEN/crc16-ccitt.o: ../Drivers/CanOPEN/crc16-ccitt.c Drivers/CanOPEN/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Drivers/CanOPEN -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/CanOPEN" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/PCA9685" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/MCP9600" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/INA226" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/STM32L4xx_HAL_Driver" -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/STM32L4xx_HAL_Driver/Inc" -I../Drivers/CMSIS/Device/ST/STM32L4xx/Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/CanOPEN/crc16-ccitt.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

