################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../driver/src/A_gpio_f407_drv.c 

OBJS += \
./driver/src/A_gpio_f407_drv.o 

C_DEPS += \
./driver/src/A_gpio_f407_drv.d 


# Each subdirectory must supply rules for building sources it contributes
driver/src/%.o: ../driver/src/%.c driver/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F407ZGTx -c -I../Inc -I../driver/inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-driver-2f-src

clean-driver-2f-src:
	-$(RM) ./driver/src/A_gpio_f407_drv.d ./driver/src/A_gpio_f407_drv.o

.PHONY: clean-driver-2f-src

