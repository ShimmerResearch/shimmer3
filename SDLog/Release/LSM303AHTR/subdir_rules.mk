################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
LSM303AHTR/lsm303ahtr.obj: ../LSM303AHTR/lsm303ahtr.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: MSP430 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-msp430_4.4.8/bin/cl430" -vmspx --abi=coffabi --code_model=large --data_model=restricted --near_data=globals -O0 --opt_for_speed=5 --use_hw_mpy=F5 --include_path="C:/ti/ccsv7/ccs_base/msp430/include" --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-msp430_4.4.8/include" --symdebug:none --gcc --define=REMOVE_LOGGING --define=USE_DMP --define=MPL_LOG_NDEBUG=1 --define=CONFIG_INTERFACE_USB --define=MPU9150 --define=I2C_B0 --define=EMPL --define=EMPL_TARGET_MSP430 --define=__MSP430F5437A__ --display_error_number --diag_warning=225 --silicon_errata=CPU21 --silicon_errata=CPU22 --silicon_errata=CPU23 --silicon_errata=CPU40 --printf_support=minimal --preproc_with_compile --preproc_dependency="LSM303AHTR/lsm303ahtr.d" --obj_directory="LSM303AHTR" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


