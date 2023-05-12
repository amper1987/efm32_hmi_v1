################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/SiliconLabs/SimplicityStudio/v5/developer/sdks/gecko_sdk_suite/v3.2/platform/Device/SiliconLabs/EFM32GG11B/Source/GCC/startup_efm32gg11b.c 

OBJS += \
./gecko_sdk_3.2.3/platform/Device/SiliconLabs/EFM32GG11B/Source/GCC/startup_efm32gg11b.o 

C_DEPS += \
./gecko_sdk_3.2.3/platform/Device/SiliconLabs/EFM32GG11B/Source/GCC/startup_efm32gg11b.d 


# Each subdirectory must supply rules for building sources it contributes
gecko_sdk_3.2.3/platform/Device/SiliconLabs/EFM32GG11B/Source/GCC/startup_efm32gg11b.o: C:/SiliconLabs/SimplicityStudio/v5/developer/sdks/gecko_sdk_suite/v3.2/platform/Device/SiliconLabs/EFM32GG11B/Source/GCC/startup_efm32gg11b.c gecko_sdk_3.2.3/platform/Device/SiliconLabs/EFM32GG11B/Source/GCC/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DDEBUG=1' '-DDEBUG_EFM=1' '-DEFM32GG11B820F2048GL192=1' '-DSL_COMPONENT_CATALOG_PRESENT=1' -I"C:\Users\igorz\SimplicityStudio\v5_workspace\hmi_v1\config" -I"C:\Users\igorz\SimplicityStudio\v5_workspace\hmi_v1\gecko_sdk_3.2.3\platform\emlib\src" -I"C:\Users\igorz\SimplicityStudio\v5_workspace\hmi_v1\gecko_sdk_3.2.3\platform\emlib\inc" -I"C:\Users\igorz\SimplicityStudio\v5_workspace\hmi_v1\autogen" -I"C:\Users\igorz\SimplicityStudio\v5_workspace\hmi_v1" -I"C:/SiliconLabs/SimplicityStudio/v5/developer/sdks/gecko_sdk_suite/v3.2//platform/Device/SiliconLabs/EFM32GG11B/Include" -I"C:/SiliconLabs/SimplicityStudio/v5/developer/sdks/gecko_sdk_suite/v3.2//platform/CMSIS/Include" -I"C:/SiliconLabs/SimplicityStudio/v5/developer/sdks/gecko_sdk_suite/v3.2//platform/service/device_init/inc" -I"C:/SiliconLabs/SimplicityStudio/v5/developer/sdks/gecko_sdk_suite/v3.2//platform/emlib/inc" -I"C:/SiliconLabs/SimplicityStudio/v5/developer/sdks/gecko_sdk_suite/v3.2//platform/common/toolchain/inc" -I"C:/SiliconLabs/SimplicityStudio/v5/developer/sdks/gecko_sdk_suite/v3.2//platform/service/system/inc" -I"C:/SiliconLabs/SimplicityStudio/v5/developer/sdks/gecko_sdk_suite/v3.2//platform/common/inc" -Os -Wall -Wextra -mno-sched-prolog -fno-builtin -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -c -fmessage-length=0 -MMD -MP -MF"gecko_sdk_3.2.3/platform/Device/SiliconLabs/EFM32GG11B/Source/GCC/startup_efm32gg11b.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


