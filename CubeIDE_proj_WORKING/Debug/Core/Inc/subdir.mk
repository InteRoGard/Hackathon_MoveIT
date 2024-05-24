################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/ds18b20.c \
../Core/Inc/onewire.c 

OBJS += \
./Core/Inc/ds18b20.o \
./Core/Inc/onewire.o 

C_DEPS += \
./Core/Inc/ds18b20.d \
./Core/Inc/onewire.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/%.o Core/Inc/%.su Core/Inc/%.cyclo: ../Core/Inc/%.c Core/Inc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103x6 -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Inc

clean-Core-2f-Inc:
	-$(RM) ./Core/Inc/ds18b20.cyclo ./Core/Inc/ds18b20.d ./Core/Inc/ds18b20.o ./Core/Inc/ds18b20.su ./Core/Inc/onewire.cyclo ./Core/Inc/onewire.d ./Core/Inc/onewire.o ./Core/Inc/onewire.su

.PHONY: clean-Core-2f-Inc

