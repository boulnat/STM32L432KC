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
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -c -I"C:/Users/sajl9/OneDrive/Documents/GitHub/STM32L432KC/MB-synthetiseur/Drivers/INA226" -x assembler-with-cpp -MMD -MP -MF"Core/Startup/startup_stm32l432kcux.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

