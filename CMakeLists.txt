cmake_minimum_required(VERSION 3.22)

# 项目基本配置
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_PROJECT_NAME C_Board_Framework)
project(${CMAKE_PROJECT_NAME} C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 硬件架构配置 (Cortex-M4)
set(CPU_FLAGS -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)

# ---------- 收集主框架源码（排除 User/App/，避免与 App_Module 重复） ----------
file(GLOB_RECURSE ALL_USER_SOURCES CONFIGURE_DEPENDS "User/*.c" "User/*.cpp" "User/*.s")
set(USER_SOURCES ${ALL_USER_SOURCES})
list(FILTER USER_SOURCES EXCLUDE REGEX "/User/App/")


# ---------- 收集所有头文件目录（不排除 App，因为主框架需要用） ----------
file(GLOB_RECURSE ALL_H_FILES CONFIGURE_DEPENDS "User/*.h" "User/*.hpp" "Core/*.h")
set(ALL_INCLUDE_DIRS "")
foreach(FILE_PATH ${ALL_H_FILES})
    get_filename_component(DIR_PATH ${FILE_PATH} DIRECTORY)
    list(APPEND ALL_INCLUDE_DIRS ${DIR_PATH})
endforeach()
list(REMOVE_DUPLICATES ALL_INCLUDE_DIRS)
list(FILTER ALL_INCLUDE_DIRS EXCLUDE REGEX "/User/App/")

add_executable(${CMAKE_PROJECT_NAME} ${USER_SOURCES})


target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
        ${ALL_INCLUDE_DIRS}
        Core/Inc
        Drivers/CMSIS/Include
        Drivers/CMSIS/DSP/Include
        Drivers/CMSIS/DSP/PrivateInclude     # DSP 内部头文件（如 arm_sorting.h）
        Drivers/CMSIS/Device/ST/STM32F4xx/Include
        Drivers/STM32F4xx_HAL_Driver/Inc
)

# 宏定义
target_compile_definitions(${CMAKE_PROJECT_NAME} PUBLIC
        STM32F407xx
        USE_HAL_DRIVER
        ARM_MATH_CM4
        __FPU_PRESENT=1U
        HAL_ASSERT_DISABLE=1
        ARM_MATH_MATRIX_CHECK=0
        ARM_MATH_ROUNDING=0
        $<$<CONFIG:Debug>:DEBUG_MODE>
        $<$<CONFIG:Release>:RELEASE_MODE>
)

# 编译选项
target_compile_options(${CMAKE_PROJECT_NAME} PUBLIC
        ${CPU_FLAGS}
        -Wall -Wextra -Wno-comment -Wno-unused-parameter -Wno-unused-but-set-variable
        -Werror=implicit-function-declaration -Werror=incompatible-pointer-types
        -fdata-sections -ffunction-sections
        $<$<CONFIG:Debug>:-Og -g3 -fno-omit-frame-pointer -fno-lto>
        $<$<CONFIG:Release>:-O3 -g3 -funroll-loops>
)

# 链接配置
target_link_options(${CMAKE_PROJECT_NAME} PUBLIC
        ${CPU_FLAGS}
        -Wl,--gc-sections
        -Wl,--print-memory-usage
        -Wl,--relax
        $<$<CONFIG:Release>:-flto=auto>
)
# ---------- STM32CubeMX 模块 ----------


# ---------- App 模块 ----------
set(ACTIVE_APP "1_Catapult_Hero")

add_subdirectory(cmake/stm32cubemx)
add_subdirectory(User/App/${ACTIVE_APP})

# ---------- CMSIS-DSP 源码编译（仅 atan2 及依赖，适用于 M4） ----------
set(CMSIS_DSP_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/DSP/Source")
file(GLOB CMSIS_DSP_SOURCES
        "${CMSIS_DSP_SOURCE_DIR}/FastMathFunctions/*.c"
        "${CMSIS_DSP_SOURCE_DIR}/CommonTables/*.c"
)
add_library(cmsis_dsp STATIC ${CMSIS_DSP_SOURCES})
target_include_directories(cmsis_dsp PUBLIC
        Drivers/CMSIS/DSP/Include
        Drivers/CMSIS/DSP/PrivateInclude
        Drivers/CMSIS/Include
)

# ---------- 链接 ----------

target_link_libraries(${CMAKE_PROJECT_NAME}
        stm32cubemx
        cmsis_dsp
)

# 后处理
set(HEX_FILE ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> "${HEX_FILE}"
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> "${BIN_FILE}"
        COMMENT "Generating HEX and BIN files..."
)