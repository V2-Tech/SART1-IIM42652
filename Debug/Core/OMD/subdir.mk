################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/OMD/DAC7571.c \
../Core/OMD/IIM42652.c \
../Core/OMD/RGB.c \
../Core/OMD/USBCommProtocol.c \
../Core/OMD/cobstranscoder.c \
../Core/OMD/common.c \
../Core/OMD/flash_manager.c \
../Core/OMD/vc_vector.c 

C_DEPS += \
./Core/OMD/DAC7571.d \
./Core/OMD/IIM42652.d \
./Core/OMD/RGB.d \
./Core/OMD/USBCommProtocol.d \
./Core/OMD/cobstranscoder.d \
./Core/OMD/common.d \
./Core/OMD/flash_manager.d \
./Core/OMD/vc_vector.d 

OBJS += \
./Core/OMD/DAC7571.o \
./Core/OMD/IIM42652.o \
./Core/OMD/RGB.o \
./Core/OMD/USBCommProtocol.o \
./Core/OMD/cobstranscoder.o \
./Core/OMD/common.o \
./Core/OMD/flash_manager.o \
./Core/OMD/vc_vector.o 


# Each subdirectory must supply rules for building sources it contributes
Core/OMD/%.o Core/OMD/%.su: ../Core/OMD/%.c Core/OMD/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F411xE -DSTM32_THREAD_SAFE_STRATEGY=4 -c -I../Core/Inc -I"C:/Users/Valerio.Mazzoni/Desktop/Progetti/Accelerometro OMD/Firmware/_STM32/AC1v2/Core/OMD" -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/ARM/DSP/Inc -I../Core/ThreadSafe -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-OMD

clean-Core-2f-OMD:
	-$(RM) ./Core/OMD/DAC7571.d ./Core/OMD/DAC7571.o ./Core/OMD/DAC7571.su ./Core/OMD/IIM42652.d ./Core/OMD/IIM42652.o ./Core/OMD/IIM42652.su ./Core/OMD/RGB.d ./Core/OMD/RGB.o ./Core/OMD/RGB.su ./Core/OMD/USBCommProtocol.d ./Core/OMD/USBCommProtocol.o ./Core/OMD/USBCommProtocol.su ./Core/OMD/cobstranscoder.d ./Core/OMD/cobstranscoder.o ./Core/OMD/cobstranscoder.su ./Core/OMD/common.d ./Core/OMD/common.o ./Core/OMD/common.su ./Core/OMD/flash_manager.d ./Core/OMD/flash_manager.o ./Core/OMD/flash_manager.su ./Core/OMD/vc_vector.d ./Core/OMD/vc_vector.o ./Core/OMD/vc_vector.su

.PHONY: clean-Core-2f-OMD

