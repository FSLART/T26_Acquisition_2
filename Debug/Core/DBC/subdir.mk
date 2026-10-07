################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/DBC/autonomous_t26.c \
../Core/DBC/data_t26.c \
../Core/DBC/powertrain_t26.c 

OBJS += \
./Core/DBC/autonomous_t26.o \
./Core/DBC/data_t26.o \
./Core/DBC/powertrain_t26.o 

C_DEPS += \
./Core/DBC/autonomous_t26.d \
./Core/DBC/data_t26.d \
./Core/DBC/powertrain_t26.d 


# Each subdirectory must supply rules for building sources it contributes
Core/DBC/%.o Core/DBC/%.su Core/DBC/%.cyclo: ../Core/DBC/%.c Core/DBC/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F412Rx -c -I../Core/Inc -I../Core/DBC -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-DBC

clean-Core-2f-DBC:
	-$(RM) ./Core/DBC/autonomous_t26.cyclo ./Core/DBC/autonomous_t26.d ./Core/DBC/autonomous_t26.o ./Core/DBC/autonomous_t26.su ./Core/DBC/data_t26.cyclo ./Core/DBC/data_t26.d ./Core/DBC/data_t26.o ./Core/DBC/data_t26.su ./Core/DBC/powertrain_t26.cyclo ./Core/DBC/powertrain_t26.d ./Core/DBC/powertrain_t26.o ./Core/DBC/powertrain_t26.su

.PHONY: clean-Core-2f-DBC

