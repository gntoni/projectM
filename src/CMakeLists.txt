# locate the necessary dependencies
# add the necessary include directories
INCLUDE_DIRECTORIES(.)
# application source files
SET(sources armPWM.cpp)
# application header files
SET(headers armPWM.h)
# create the executable file
ADD_EXECUTABLE(armPWM ${sources})
# link necessary libraries
