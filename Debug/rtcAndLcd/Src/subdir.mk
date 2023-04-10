################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../rtcAndLcd/Src/rtc_ds1307.c 

OBJS += \
./rtcAndLcd/Src/rtc_ds1307.o 

C_DEPS += \
./rtcAndLcd/Src/rtc_ds1307.d 


# Each subdirectory must supply rules for building sources it contributes
rtcAndLcd/Src/%.o rtcAndLcd/Src/%.su rtcAndLcd/Src/%.cyclo: ../rtcAndLcd/Src/%.c rtcAndLcd/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F411VETx -DSTM32 -DSTM32F4 -DSTM32F411E_DISCO -c -I../Inc -I"D:/Courses/976 STM32 bare metal/CodeForThisCourse/stm32f4xx_DeviceDriver/rtcAndLcd/Inc" -I"D:/Courses/976 STM32 bare metal/CodeForThisCourse/stm32f4xx_DeviceDriver/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-rtcAndLcd-2f-Src

clean-rtcAndLcd-2f-Src:
	-$(RM) ./rtcAndLcd/Src/rtc_ds1307.cyclo ./rtcAndLcd/Src/rtc_ds1307.d ./rtcAndLcd/Src/rtc_ds1307.o ./rtcAndLcd/Src/rtc_ds1307.su

.PHONY: clean-rtcAndLcd-2f-Src

