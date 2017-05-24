################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
main.obj: ../main.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: MSP430 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-msp430_4.4.8/bin/cl430" -vmspx --abi=coffabi --code_model=large --data_model=restricted --near_data=globals -O0 --opt_for_speed=5 --use_hw_mpy=F5 --include_path="C:/ti/ccsv7/ccs_base/msp430/include" --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-msp430_4.4.8/include" --include_path="C:/Users/SOMAHONY/Documents/Github/Firmware/FW_Shimmer3/SDLog/MPU9150/core/driver/eMPL" --include_path="C:/Users/SOMAHONY/Documents/Github/Firmware/FW_Shimmer3/SDLog/MPU9150/core/driver/include" --include_path="C:/Users/SOMAHONY/Documents/Github/Firmware/FW_Shimmer3/SDLog/MPU9150/core/driver/msp430" --include_path="C:/Users/SOMAHONY/Documents/Github/Firmware/FW_Shimmer3/SDLog/MPU9150/core/driver/msp430/F5xx_F6xx_Core_Lib" --include_path="C:/Users/SOMAHONY/Documents/Github/Firmware/FW_Shimmer3/SDLog/MPU9150/core/driver/msp430/USB_eMPL" --include_path="C:/Users/SOMAHONY/Documents/Github/Firmware/FW_Shimmer3/SDLog/MPU9150" --include_path="C:/Users/SOMAHONY/Documents/Github/Firmware/FW_Shimmer3/SDLog/MPU9150/core/mpl" --include_path="C:/Users/SOMAHONY/Documents/Github/Firmware/FW_Shimmer3/SDLog/MPU9150/core/mllite" --include_path="C:/Users/SOMAHONY/Documents/Github/Firmware/FW_Shimmer3/SDLog/MPU9150/core/eMPL-hal" --symdebug:none --gcc --define=REMOVE_LOGGING --define=USE_DMP --define=MPL_LOG_NDEBUG=1 --define=CONFIG_INTERFACE_USB --define=MPU9150 --define=I2C_B0 --define=EMPL --define=EMPL_TARGET_MSP430 --define=__MSP430F5437A__ --display_error_number --diag_warning=225 --silicon_errata=CPU21 --silicon_errata=CPU22 --silicon_errata=CPU23 --silicon_errata=CPU40 --printf_support=minimal --preproc_with_compile --preproc_dependency="main.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


