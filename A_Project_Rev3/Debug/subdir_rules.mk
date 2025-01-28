################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/Program Files/ccs/tools/compiler/ti-cgt-armllvm_2.1.3.LTS/bin/tiarmclang.exe" -c @"syscfg/device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"C:/Users/yjia1/Documents/mechatronics_ccs/A_Project_Rev3" -I"C:/Users/yjia1/Documents/mechatronics_ccs/A_Project_Rev3/Debug" -I"C:/ti/mspm0_sdk_2_01_00_03/source/third_party/CMSIS/Core/Include" -I"C:/ti/mspm0_sdk_2_01_00_03/source" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)" -I"C:/Users/yjia1/Documents/mechatronics_ccs/A_Project_Rev3/Debug/syscfg"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-865109567: ../project_rev3.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"C:/ti/sysconfig_1.19.0/sysconfig_cli.bat" -s "C:/ti/mspm0_sdk_2_01_00_03/.metadata/product.json" --script "C:/Users/yjia1/Documents/mechatronics_ccs/A_Project_Rev3/project_rev3.syscfg" -o "syscfg" --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

syscfg/device_linker.cmd: build-865109567 ../project_rev3.syscfg
syscfg/device.opt: build-865109567
syscfg/device.cmd.genlibs: build-865109567
syscfg/ti_msp_dl_config.c: build-865109567
syscfg/ti_msp_dl_config.h: build-865109567
syscfg/: build-865109567

syscfg/%.o: ./syscfg/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/Program Files/ccs/tools/compiler/ti-cgt-armllvm_2.1.3.LTS/bin/tiarmclang.exe" -c @"syscfg/device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"C:/Users/yjia1/Documents/mechatronics_ccs/A_Project_Rev3" -I"C:/Users/yjia1/Documents/mechatronics_ccs/A_Project_Rev3/Debug" -I"C:/ti/mspm0_sdk_2_01_00_03/source/third_party/CMSIS/Core/Include" -I"C:/ti/mspm0_sdk_2_01_00_03/source" -gdwarf-3 -MMD -MP -MF"syscfg/$(basename $(<F)).d_raw" -MT"$(@)" -I"C:/Users/yjia1/Documents/mechatronics_ccs/A_Project_Rev3/Debug/syscfg"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

startup_mspm0g350x_ticlang.o: C:/ti/mspm0_sdk_2_01_00_03/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/Program Files/ccs/tools/compiler/ti-cgt-armllvm_2.1.3.LTS/bin/tiarmclang.exe" -c @"syscfg/device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"C:/Users/yjia1/Documents/mechatronics_ccs/A_Project_Rev3" -I"C:/Users/yjia1/Documents/mechatronics_ccs/A_Project_Rev3/Debug" -I"C:/ti/mspm0_sdk_2_01_00_03/source/third_party/CMSIS/Core/Include" -I"C:/ti/mspm0_sdk_2_01_00_03/source" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)" -I"C:/Users/yjia1/Documents/mechatronics_ccs/A_Project_Rev3/Debug/syscfg"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


