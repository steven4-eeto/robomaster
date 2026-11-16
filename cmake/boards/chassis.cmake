add_compile_definitions(ARM_MATH_CM4 ARM_MATH_MATRIX_CHECK ARM_MATH_ROUNDING)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER "C:/arm-gnu-toolchain/bin/arm-none-eabi-gcc.exe")
set(CMAKE_CXX_COMPILER "C:/arm-gnu-toolchain/bin/arm-none-eabi-g++.exe")
set(CMAKE_ASM_COMPILER "C:/arm-gnu-toolchain/bin/arm-none-eabi-gcc.exe")

set(CMAKE_C_FLAGS_INIT   "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(CMAKE_CXX_FLAGS_INIT "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(CMAKE_ASM_FLAGS_INIT "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(BOARD_CORE_DIR    "${CMAKE_CURRENT_SOURCE_DIR}/chassis/Core")
set(BOARD_DRIVERS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/chassis/Drivers")
set(BOARD_USER_FOLDERS "chassis/Instance" 
    "chassis/RobotModules" 
    "chassis/Task" 
    "chassis/src"
    "RobotComponents"
)
set(BOARD_STARTUP_DIR  "${CMAKE_CURRENT_SOURCE_DIR}/chassis")
set(BOARD_CUBEMX_DIR   "${CMAKE_CURRENT_SOURCE_DIR}/chassis/cmake/stm32cubemx")