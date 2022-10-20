################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/vl53l0x/VL53L0X.c 

OBJS += \
./Drivers/vl53l0x/VL53L0X.o 

C_DEPS += \
./Drivers/vl53l0x/VL53L0X.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/vl53l0x/%.o Drivers/vl53l0x/%.su: ../Drivers/vl53l0x/%.c Drivers/vl53l0x/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G031xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -I"G:/YandexDisk/Projects/DiLight/firmware/Drivers/vl53l0x" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-vl53l0x

clean-Drivers-2f-vl53l0x:
	-$(RM) ./Drivers/vl53l0x/VL53L0X.d ./Drivers/vl53l0x/VL53L0X.o ./Drivers/vl53l0x/VL53L0X.su

.PHONY: clean-Drivers-2f-vl53l0x

