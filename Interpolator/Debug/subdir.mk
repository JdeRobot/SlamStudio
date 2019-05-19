################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Interpolator.cpp 

OBJS += \
./Interpolator.o 

CPP_DEPS += \
./Interpolator.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -I/home/tfm3/workspace/AjusteTiempo -include/home/tfm3/workspace/AjusteTiempo/AjusteTiempo.h -include/home/tfm3/workspace/AjusteTiempo/AjusteTiempo.cpp -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


