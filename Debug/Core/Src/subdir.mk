################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/apds9250_light.c \
../Core/Src/logger.c \
../Core/Src/lsm6dsl_accel_gyro.c \
../Core/Src/main.c \
../Core/Src/max1704x_fuel_gauge.c \
../Core/Src/ms8607_pth.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h7xx.c 

OBJS += \
./Core/Src/apds9250_light.o \
./Core/Src/logger.o \
./Core/Src/lsm6dsl_accel_gyro.o \
./Core/Src/main.o \
./Core/Src/max1704x_fuel_gauge.o \
./Core/Src/ms8607_pth.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h7xx.o 

C_DEPS += \
./Core/Src/apds9250_light.d \
./Core/Src/logger.d \
./Core/Src/lsm6dsl_accel_gyro.d \
./Core/Src/main.d \
./Core/Src/max1704x_fuel_gauge.d \
./Core/Src/ms8607_pth.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h7xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DPICOSTM_COMBO_REV_B -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/apds9250_light.cyclo ./Core/Src/apds9250_light.d ./Core/Src/apds9250_light.o ./Core/Src/apds9250_light.su ./Core/Src/logger.cyclo ./Core/Src/logger.d ./Core/Src/logger.o ./Core/Src/logger.su ./Core/Src/lsm6dsl_accel_gyro.cyclo ./Core/Src/lsm6dsl_accel_gyro.d ./Core/Src/lsm6dsl_accel_gyro.o ./Core/Src/lsm6dsl_accel_gyro.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/max1704x_fuel_gauge.cyclo ./Core/Src/max1704x_fuel_gauge.d ./Core/Src/max1704x_fuel_gauge.o ./Core/Src/max1704x_fuel_gauge.su ./Core/Src/ms8607_pth.cyclo ./Core/Src/ms8607_pth.d ./Core/Src/ms8607_pth.o ./Core/Src/ms8607_pth.su ./Core/Src/stm32h7xx_hal_msp.cyclo ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_it.cyclo ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h7xx.cyclo ./Core/Src/system_stm32h7xx.d ./Core/Src/system_stm32h7xx.o ./Core/Src/system_stm32h7xx.su

.PHONY: clean-Core-2f-Src

