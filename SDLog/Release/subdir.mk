################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 

# Each subdirectory must supply rules for building sources it contributes
main.null: ../main.c
	@echo 'Building file: $<'
	@echo 'Invoking: MSP430 Compiler'
	__MSP430F5528__ REMOVE_LOGGING USE_DMP MPL_LOG_NDEBUG=1 CONFIG_INTERFACE_USB MPU9150 I2C_B0 EMPL EMPL_TARGET_MSP430 "/msp430/include" "/include" 225 "/msp430/include" "/include" "C:\Users\SOMAHONY\Documents\Github\Firmware\FW_Shimmer3\SDLog\MPU9150\core\driver\eMPL" "C:\Users\SOMAHONY\Documents\Github\Firmware\FW_Shimmer3\SDLog\MPU9150\core\driver\include" "C:\Users\SOMAHONY\Documents\Github\Firmware\FW_Shimmer3\SDLog\MPU9150\core\driver\msp430" "C:\Users\SOMAHONY\Documents\Github\Firmware\FW_Shimmer3\SDLog\MPU9150\core\driver\msp430\F5xx_F6xx_Core_Lib" "C:\Users\SOMAHONY\Documents\Github\Firmware\FW_Shimmer3\SDLog\MPU9150\core\driver\msp430\USB_eMPL" "C:\Users\SOMAHONY\Documents\Github\Firmware\FW_Shimmer3\SDLog\MPU9150" "C:\Users\SOMAHONY\Documents\Github\Firmware\FW_Shimmer3\SDLog\MPU9150\core\mpl" "C:\Users\SOMAHONY\Documents\Github\Firmware\FW_Shimmer3\SDLog\MPU9150\core\mllite" "C:\Users\SOMAHONY\Documents\Github\Firmware\FW_Shimmer3\SDLog\MPU9150\core\eMPL-hal"  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


