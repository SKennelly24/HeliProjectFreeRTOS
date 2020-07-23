################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
ence464-tivaware-freertos-example-master/ence464-tivaware-freertos-example-master/%.obj: ../ence464-tivaware-freertos-example-master/ence464-tivaware-freertos-example-master/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccs920/ccs/tools/compiler/ti-cgt-arm_18.12.4.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="C:/ti/ccs920/ccs/tools/compiler/ti-cgt-arm_18.12.4.LTS/include" --include_path="C:/ti/TivaWare_C_Series-2.1.4.178" --define=ccs="ccs" --define=PART_TM4C123GH6PM -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="ence464-tivaware-freertos-example-master/ence464-tivaware-freertos-example-master/$(basename $(<F)).d_raw" --obj_directory="ence464-tivaware-freertos-example-master/ence464-tivaware-freertos-example-master" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


