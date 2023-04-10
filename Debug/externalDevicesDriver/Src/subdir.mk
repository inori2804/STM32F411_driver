################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../externalDevicesDriver/Src/i2c_lcd_esp32s2.c \
../externalDevicesDriver/Src/rtc_ds1307.c 

OBJS += \
./externalDevicesDriver/Src/i2c_lcd_esp32s2.o \
./externalDevicesDriver/Src/rtc_ds1307.o 

C_DEPS += \
./externalDevicesDriver/Src/i2c_lcd_esp32s2.d \
./externalDevicesDriver/Src/rtc_ds1307.d 


# Each subdirectory must supply rules for building sources it contributes
externalDevicesDriver/Src/%.o externalDevicesDriver/Src/%.su externalDevicesDriver/Src/%.cyclo: ../externalDevicesDriver/Src/%.c externalDevicesDriver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F411VETx -DSTM32 -DSTM32F4 -DSTM32F411E_DISCO -c -I"D:/Courses/976 STM32 bare metal/CodeForThisCourse/stm32f4xx_DeviceDriver/externalDevicesDriver/Inc" -I../Inc -I"D:/Courses/976 STM32 bare metal/CodeForThisCourse/stm32f4xx_DeviceDriver/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-externalDevicesDriver-2f-Src

clean-externalDevicesDriver-2f-Src:
	-$(RM) ./externalDevicesDriver/Src/i2c_lcd_esp32s2.cyclo ./externalDevicesDriver/Src/i2c_lcd_esp32s2.d ./externalDevicesDriver/Src/i2c_lcd_esp32s2.o ./externalDevicesDriver/Src/i2c_lcd_esp32s2.su ./externalDevicesDriver/Src/rtc_ds1307.cyclo ./externalDevicesDriver/Src/rtc_ds1307.d ./externalDevicesDriver/Src/rtc_ds1307.o ./externalDevicesDriver/Src/rtc_ds1307.su

.PHONY: clean-externalDevicesDriver-2f-Src

