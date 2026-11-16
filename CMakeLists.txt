# ##############################################################################
# #################        CMake Template (CUSTOM)       ######################
# #################    Copyright (c) 2026 Hello World    ######################
# ##############################################################################

# Set the system name and version
set(CMAKE_SYSTEM_VERSION 1)

# Specify the minimum required version of CMake
cmake_minimum_required(VERSION 3.22)

if(NOT DEFINED BOARD)
  set(BOARD "chassis" CACHE STRING "Target board: chassis or gimbal")
endif()
#set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/boards/${BOARD}.cmake" CACHE FILEPATH "Toolchain file")

# Set the C++ and C standards
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Set the library path
set(CMAKE_LIBRARY_PATH "${CMAKE_CURRENT_SOURCE_DIR}/Lib")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# ########################## USER CONFIG SECTION ##############################
# Set the project name and the languages used
project(${BOARD} C CXX ASM) # TODO: change project name here

# Specify user folders
 # TODO Add your own user folders here

# Specify the path to the HW-Components directory
# set(HWC_DIR "HW-Components") # TODO: Set your own HW-Components path here

# # Include utility functions and default configuration
# include("${HWC_DIR}/cmake/utils/function_tools.cmake")
# include("${HWC_DIR}/config.cmake") # Default configuration

# TODO: Overwrite default configuration instead of changing it in file
# tools
# set(use_hwcomponents_tools ON)
# set(use_prebuilt_hwcomponents_tools OFF)
# set(... ON)

# TODO: Add your own `config.cmake` file or set your own configuration here

# Enable preprocessing for assembler files
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

# Disable some warnings
set(COM_FLAGS "-Wno-unused-parameter -Wno-missing-field-initializers -Wno-pedantic -Wno-unknown-pragmas -Wno-comment")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COM_FLAGS} -g -gdwarf-4")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COM_FLAGS} -Wno-reorder -g -gdwarf-4")

##ADD_FUNCTIONS
function(search_incs_recurse root_dir res_list)
  get_filename_component(root_name ${root_dir} NAME)
  set(_${root_name}_incs ${root_dir})
  file(
    GLOB children
    RELATIVE ${root_dir}
    ${root_dir}/*)
  foreach(child ${children})
    set(sub_dir ${root_dir}/${child})
    if(IS_DIRECTORY ${sub_dir})
      get_filename_component(child_name ${child} NAME)
      set(__${child_name}_incs)
      search_incs_recurse(${sub_dir} __${child_name}_incs)
      list(APPEND _${root_name}_incs ${__${child_name}_incs})
    endif()
  endforeach()
  set(${res_list}
      ${_${root_name}_incs}
      PARENT_SCOPE)
  return()
endfunction()
##

# #################### ADD LIBRARIES AND EXECUTABLE SECTION ####################
# Initialize source and include lists
set(project_srcs)
set(project_incs)

# Search for include files and source files in the Core directory
search_incs_recurse("${BOARD_CORE_DIR}" core_incs)
file(GLOB_RECURSE core_srcs "${BOARD_CORE_DIR}/*.c" "${BOARD_CORE_DIR}/*.cpp") 
# 排除所有形如 system_stm32*.c 的文件，提升通用性
# TODO:若使用低版本的cubemx，需要将这两行注释掉
file(GLOB EXCLUDE_SYSTEM_SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/system_stm32*.c")
list(REMOVE_ITEM core_srcs ${EXCLUDE_SYSTEM_SRC_FILES})

list(APPEND project_srcs ${core_srcs})
list(APPEND project_incs ${core_incs})

# Search for include files and source files in the Drivers directory
search_incs_recurse("${BOARD_DRIVERS_DIR}" drivers_incs)
file(GLOB_RECURSE drivers_srcs "${BOARD_DRIVERS_DIR}/*.c" "${BOARD_DRIVERS_DIR}/*.cpp")
list(APPEND project_incs ${drivers_incs})

# For each user folder, search for include files and source files
foreach(user_folder ${BOARD_USER_FOLDERS})
  search_incs_recurse("${CMAKE_CURRENT_SOURCE_DIR}/${user_folder}"
                      ${user_folder}_incs)
  file(GLOB_RECURSE ${user_folder}_srcs "${user_folder}/*.c" "${user_folder}/*.cpp")
  list(APPEND project_incs ${${user_folder}_incs})
  list(APPEND project_srcs ${${user_folder}_srcs})
endforeach()

# Add a static library for the drivers
add_library(${BOARD}_drivers STATIC ${drivers_srcs})
target_include_directories(${BOARD}_drivers PRIVATE ${core_incs} PUBLIC ${drivers_incs})

if(BOARD STREQUAL "chassis")

target_include_directories(${BOARD}_drivers PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/Chassis/inc
)

elseif(BOARD STREQUAL "gimbal")
    target_include_directories(${BOARD}_drivers PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include
        ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
        ${CMAKE_CURRENT_SOURCE_DIR}/Gimbal/inc
    )

endif()
# Add an executable for the project
file(GLOB_RECURSE startup_file "${BOARD_STARTUP_DIR}/*.s")

if(BOARD STREQUAL "chassis")

add_executable(${PROJECT_NAME} ${project_srcs} ${startup_file}
    chassis/third_party/SEGGER/RTT/SEGGER_RTT.c
)

elseif(BOARD STREQUAL "gimbal")
    add_executable(${PROJECT_NAME} ${project_srcs} ${startup_file}
        gimbal/third_party/SEGGER/RTT/SEGGER_RTT.c
    )

endif()

set_target_properties(${PROJECT_NAME} PROPERTIES SUFFIX ".elf")
message(STATUS "Project name: ${PROJECT_NAME}")

target_compile_definitions(${PROJECT_NAME} PRIVATE USE_HAL_TICK)
#
if(BOARD STREQUAL "chassis")

target_include_directories(chassis PRIVATE
    Chassis/third_party/dsp/CMSIS-DSP/Include
    # 注意：下面这个路径非常关键！请确认你的工程里有这个文件夹
    Drivers/CMSIS/Include  
    Drivers/CMSIS/Device/ST/STM32F4xx/Include # (STM32F4的底层头文件)
)

target_compile_definitions(chassis PRIVATE
    ARM_MATH_CM4
    ARM_MATH_MATRIX_CHECK
    ARM_MATH_ROUNDING
)

target_sources(chassis PRIVATE
    Chassis/third_party/dsp/CMSIS-DSP/Source/BasicMathFunctions/arm_scale_f32.c
    Chassis/third_party/dsp/CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_f32.c
    Chassis/third_party/dsp/CMSIS-DSP/Source/MatrixFunctions/arm_mat_vec_mult_f32.c
    Chassis/third_party/dsp/CMSIS-DSP/Source/FastMathFunctions/arm_atan2_f32.c
    Chassis/third_party/dsp/CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_product_single_f32.c
    Chassis/third_party/dsp/CMSIS-DSP/Source/BasicMathFunctions/arm_add_f32.c
)
#

target_link_options(chassis PRIVATE
    -T${CMAKE_SOURCE_DIR}/chassis/STM32F407IGHx_FLASH.ld
)

elseif(BOARD STREQUAL "gimbal")
    target_include_directories(${PROJECT_NAME} PRIVATE
        Gimbal/third_party/dsp/CMSIS-DSP/Include
        Drivers/CMSIS/Include
        Drivers/CMSIS/Device/ST/STM32F4xx/Include
    )

    target_compile_definitions(${PROJECT_NAME} PRIVATE
        ARM_MATH_CM4
        ARM_MATH_MATRIX_CHECK
        ARM_MATH_ROUNDING
    )

    target_sources(${PROJECT_NAME} PRIVATE
        Gimbal/third_party/dsp/CMSIS-DSP/Source/BasicMathFunctions/arm_scale_f32.c
        Gimbal/third_party/dsp/CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_f32.c
        Gimbal/third_party/dsp/CMSIS-DSP/Source/MatrixFunctions/arm_mat_vec_mult_f32.c
        Gimbal/third_party/dsp/CMSIS-DSP/Source/FastMathFunctions/arm_atan2_f32.c
        Gimbal/third_party/dsp/CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_product_single_f32.c
        Gimbal/third_party/dsp/CMSIS-DSP/Source/BasicMathFunctions/arm_add_f32.c
    )

    target_link_options(${PROJECT_NAME} PRIVATE
        -T${CMAKE_SOURCE_DIR}/gimbal/STM32F407IGHx_FLASH.ld
    )

endif()
# --- 手动添加芯片宏定义 ---
# 添加通用的宏
target_compile_definitions(${PROJECT_NAME} PRIVATE USE_HAL_DRIVER DEBUG)

# 针对不同板子添加不同的芯片型号宏（请根据你的实际芯片修改！）
# 假设你的底盘和云台都是 STM32F407，如果是其他型号请自行更改
if(BOARD STREQUAL "chassis")
    target_compile_definitions(${PROJECT_NAME} PRIVATE STM32F407xx)
    set(STM32_DEVICE "STM32F4")
elseif(BOARD STREQUAL "gimbal")
    target_compile_definitions(${PROJECT_NAME} PRIVATE STM32F407xx) # 如果云台是 F103，这里换成 STM32F103xx
    set(STM32_DEVICE "STM32F4") # 如果云台是 F103，这里换成 STM32F103
endif()

# 自动推导并设置 HAL 头文件名宏
if(STM32_DEVICE)
    string(TOLOWER ${STM32_DEVICE} stm32_hal_filename)
    string(CONCAT stm32_hal_filename ${stm32_hal_filename} "xx_hal.h")
    message(STATUS "HAL file name: ${stm32_hal_filename}")
    add_definitions(-DSTM32_HAL_FILENAME="${stm32_hal_filename}")
endif()

# Add the HW-Components directory as a subdirectory
# add_subdirectory(${HWC_DIR})

# Add the project includes to the executable
target_include_directories(${PROJECT_NAME} PUBLIC ${project_incs})
#target_include_directories(${PROJECT_NAME} PUBLIC ${${HWC_LIB_PREFIX}_incs})
if(BOARD STREQUAL "chassis")

target_include_directories(${PROJECT_NAME} PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/robomaster/Chassis/inc"
)
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Chassis/third_party/dsp/CMSIS-DSP/Include
)

elseif(BOARD STREQUAL "gimbal")
    target_include_directories(${PROJECT_NAME} PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/robomaster/Gimbal/inc"
    )
    target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/Gimbal/third_party/dsp/CMSIS-DSP/Include
    )

endif()
# Link the drivers library and the HW-Components library to the executable
target_link_libraries(${PROJECT_NAME}  PRIVATE ${BOARD}_drivers)
target_link_libraries(${PROJECT_NAME}  ${${HWC_LIB_PREFIX}_libs})

message(STATUS "Drivers link libs: ${chassis_drivers_LINK_LIBRARIES}")
# Define the output hex and bin files
set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

# Add a post-build command to generate the hex and bin files
add_custom_command(
  TARGET ${PROJECT_NAME}
  POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}> ${HEX_FILE}
  COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}>
          ${BIN_FILE}
  COMMENT "Building ${HEX_FILE} Building ${BIN_FILE}")