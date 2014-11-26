ADD_SUBDIRECTORY(gpio)

# locate the necessary dependencies
FIND_PACKAGE(iriutils REQUIRED)

# add the necessary include directories
INCLUDE_DIRECTORIES(.)
INCLUDE_DIRECTORIES(./gpio)
INCLUDE_DIRECTORIES(${iriutils_INCLUDE_DIR})

# application source files
SET(sources armPWM.cpp)

# application header files
SET(headers armPWM.h)

# create the executable file
ADD_EXECUTABLE(armPWM ${sources})

# link necessary libraries
TARGET_LINK_LIBRARIES(armPWM ${iriutils_LIBRARY})
TARGET_LINK_LIBRARIES(armPWM bbb_gpio)
