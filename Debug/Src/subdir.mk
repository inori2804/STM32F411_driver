################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/esp32s2_i2c_lcd.c \
../Src/sysmem.c 

OBJS += \
./Src/esp32s2_i2c_lcd.o \
./Src/sysmem.o 

C_DEPS += \
./Src/esp32s2_i2c_lcd.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F411VETx -DSTM32 -DSTM32F4 -DSTM32F411E_DISCO -c -I"D:/Courses/976 STM32 bare metal/CodeForThisCourse/stm32f4xx_DeviceDriver/externalDevicesDriver/Inc" -I../Inc -I"D:/Courses/976 STM32 bare metal/CodeForThisCourse/stm32f4xx_DeviceDriver/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/esp32s2_i2c_lcd.cyclo ./Src/esp32s2_i2c_lcd.d ./Src/esp32s2_i2c_lcd.o ./Src/esp32s2_i2c_lcd.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

