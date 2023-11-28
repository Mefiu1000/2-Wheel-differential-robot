################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/my_lib/Button.c \
../Core/Src/my_lib/HCSR04p.c \
../Core/Src/my_lib/L298N.c \
../Core/Src/my_lib/RingBuffer.c \
../Core/Src/my_lib/Robot.c \
../Core/Src/my_lib/complex_parser.c \
../Core/Src/my_lib/utils.c 

OBJS += \
./Core/Src/my_lib/Button.o \
./Core/Src/my_lib/HCSR04p.o \
./Core/Src/my_lib/L298N.o \
./Core/Src/my_lib/RingBuffer.o \
./Core/Src/my_lib/Robot.o \
./Core/Src/my_lib/complex_parser.o \
./Core/Src/my_lib/utils.o 

C_DEPS += \
./Core/Src/my_lib/Button.d \
./Core/Src/my_lib/HCSR04p.d \
./Core/Src/my_lib/L298N.d \
./Core/Src/my_lib/RingBuffer.d \
./Core/Src/my_lib/Robot.d \
./Core/Src/my_lib/complex_parser.d \
./Core/Src/my_lib/utils.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/my_lib/%.o Core/Src/my_lib/%.su Core/Src/my_lib/%.cyclo: ../Core/Src/my_lib/%.c Core/Src/my_lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-my_lib

clean-Core-2f-Src-2f-my_lib:
	-$(RM) ./Core/Src/my_lib/Button.cyclo ./Core/Src/my_lib/Button.d ./Core/Src/my_lib/Button.o ./Core/Src/my_lib/Button.su ./Core/Src/my_lib/HCSR04p.cyclo ./Core/Src/my_lib/HCSR04p.d ./Core/Src/my_lib/HCSR04p.o ./Core/Src/my_lib/HCSR04p.su ./Core/Src/my_lib/L298N.cyclo ./Core/Src/my_lib/L298N.d ./Core/Src/my_lib/L298N.o ./Core/Src/my_lib/L298N.su ./Core/Src/my_lib/RingBuffer.cyclo ./Core/Src/my_lib/RingBuffer.d ./Core/Src/my_lib/RingBuffer.o ./Core/Src/my_lib/RingBuffer.su ./Core/Src/my_lib/Robot.cyclo ./Core/Src/my_lib/Robot.d ./Core/Src/my_lib/Robot.o ./Core/Src/my_lib/Robot.su ./Core/Src/my_lib/complex_parser.cyclo ./Core/Src/my_lib/complex_parser.d ./Core/Src/my_lib/complex_parser.o ./Core/Src/my_lib/complex_parser.su ./Core/Src/my_lib/utils.cyclo ./Core/Src/my_lib/utils.d ./Core/Src/my_lib/utils.o ./Core/Src/my_lib/utils.su

.PHONY: clean-Core-2f-Src-2f-my_lib

