# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../ControllerTest.cpp \
../ObstaclePredictorTest.cpp \
../RoutePlannerTest.cpp \
../main.cpp 

OBJS += \
./ControllerTest.o \
./ObstaclePredictorTest.o \
./RoutePlannerTest.o \
./main.o 

CPP_DEPS += \
./ControllerTest.d \
./ObstaclePredictorTest.d \
./RoutePlannerTest.d \
./main.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++0x -I"../../Auxiliary" -I"../Auxiliary_test" -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


