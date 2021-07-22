################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32l432kcux.s 

OBJS += \
./Core/Startup/startup_stm32l432kcux.o 

S_DEPS += \
./Core/Startup/startup_stm32l432kcux.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/startup_stm32l432kcux.o: ../Core/Startup/startup_stm32l432kcux.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -c -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/INA226" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/STM32L4xx_HAL_Driver" -I"/home/boulnat/STM32L432KC/MB-synthetiseur/Drivers/STM32L4xx_HAL_Driver/Inc" -I../Drivers/CMSIS/Device/ST/STM32L4xx/Inc -x assembler-with-cpp -MMD -MP -MF"Core/Startup/startup_stm32l432kcux.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

