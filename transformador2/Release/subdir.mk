################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Point3D.cpp \
../RegisterModule.cpp \
../Transformador.cpp \
../transformaXYZ.cpp 

OBJS += \
./Point3D.o \
./RegisterModule.o \
./Transformador.o \
./transformaXYZ.o 

CPP_DEPS += \
./Point3D.d \
./RegisterModule.d \
./Transformador.d \
./transformaXYZ.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


