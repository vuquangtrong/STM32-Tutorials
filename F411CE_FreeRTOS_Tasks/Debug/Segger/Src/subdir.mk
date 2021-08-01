################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Segger/Src/SEGGER_RTT.c \
../Segger/Src/SEGGER_RTT_printf.c \
../Segger/Src/SEGGER_SYSVIEW.c 

S_UPPER_SRCS += \
../Segger/Src/SEGGER_RTT_ASM_ARMv7M.S 

OBJS += \
./Segger/Src/SEGGER_RTT.o \
./Segger/Src/SEGGER_RTT_ASM_ARMv7M.o \
./Segger/Src/SEGGER_RTT_printf.o \
./Segger/Src/SEGGER_SYSVIEW.o 

S_UPPER_DEPS += \
./Segger/Src/SEGGER_RTT_ASM_ARMv7M.d 

C_DEPS += \
./Segger/Src/SEGGER_RTT.d \
./Segger/Src/SEGGER_RTT_printf.d \
./Segger/Src/SEGGER_SYSVIEW.d 


# Each subdirectory must supply rules for building sources it contributes
Segger/Src/%.o: ../Segger/Src/%.c Segger/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Segger/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Segger/Src/%.o: ../Segger/Src/%.S Segger/Src/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I../Segger/Inc -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

