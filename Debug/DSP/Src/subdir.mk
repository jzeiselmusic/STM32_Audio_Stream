################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../DSP/Src/arm_dot_prod_f32.c \
../DSP/Src/arm_float_to_q31.c 

OBJS += \
./DSP/Src/arm_dot_prod_f32.o \
./DSP/Src/arm_float_to_q31.o 

C_DEPS += \
./DSP/Src/arm_dot_prod_f32.d \
./DSP/Src/arm_float_to_q31.d 


# Each subdirectory must supply rules for building sources it contributes
DSP/Src/%.o DSP/Src/%.su: ../DSP/Src/%.c DSP/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../DSP/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-DSP-2f-Src

clean-DSP-2f-Src:
	-$(RM) ./DSP/Src/arm_dot_prod_f32.d ./DSP/Src/arm_dot_prod_f32.o ./DSP/Src/arm_dot_prod_f32.su ./DSP/Src/arm_float_to_q31.d ./DSP/Src/arm_float_to_q31.o ./DSP/Src/arm_float_to_q31.su

.PHONY: clean-DSP-2f-Src

