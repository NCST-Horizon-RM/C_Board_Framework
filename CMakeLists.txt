cmake_minimum_required(VERSION 3.22)

# 交叉编译时避免 try_compile 链接错误
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# 项目名称
set(CMAKE_PROJECT_NAME C_Board_Framework)

# 启用 C、C++ 和汇编语言
project(${CMAKE_PROJECT_NAME} C CXX ASM)

# 设置语言标准
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 默认构建类型（若未定义则设为 Debug）
#if(NOT CMAKE_BUILD_TYPE)
#    set(CMAKE_BUILD_TYPE "Debug")
#endif()
#message("Build type: " ${CMAKE_BUILD_TYPE})

# 导出编译命令供 clangd 等工具使用
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 硬件架构配置
set(CPU_FLAGS -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)

# 自动搜索用户源文件
file(GLOB_RECURSE USER_SOURCES
        CONFIGURE_DEPENDS
        "User/*.c" "User/*.cpp" "User/*.s"
)

# 创建可执行目标
add_executable(${CMAKE_PROJECT_NAME} ${USER_SOURCES})

# 自动收集头文件目录
file(GLOB_RECURSE ALL_H_FILES CONFIGURE_DEPENDS "User/*.h" "User/*.hpp" "Core/*.h")
set(ALL_INCLUDE_DIRS "")
foreach(FILE_PATH ${ALL_H_FILES})
    get_filename_component(DIR_PATH ${FILE_PATH} DIRECTORY)
    list(APPEND ALL_INCLUDE_DIRS ${DIR_PATH})
endforeach()
list(REMOVE_DUPLICATES ALL_INCLUDE_DIRS)

# 添加包含路径
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC

        Core/Inc
        Drivers/CMSIS/Include
        Drivers/CMSIS/DSP/Include
        Drivers/CMSIS/Device/ST/STM32F4xx/Include
        Drivers/STM32F4xx_HAL_Driver/Inc
        ${ALL_INCLUDE_DIRS}# 用户自定义包含路径可追加于此
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
        # 用户自定义宏可追加于此
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

# 链接选项
target_link_options(${CMAKE_PROJECT_NAME} PUBLIC
        ${CPU_FLAGS}
        -Wl,--gc-sections
        -Wl,--print-memory-usage
        -Wl,--relax
        $<$<CONFIG:Release>:-flto=auto>
)

# 链接目录
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # 添加用户自定义库搜索路径
)

# 自动查找 DSP 静态库
file(GLOB_RECURSE LIB_FILES "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/*.a" "${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/*.a")
set(DSP_LIB "")
foreach(LIB ${LIB_FILES})
    if(LIB MATCHES "arm_cortexM4lf_math")   # 请匹配您的 DSP 库文件名
        set(DSP_LIB ${LIB})
        break()
    endif()
endforeach()

# 包含 STM32CubeMX 生成的 CMake 模块
add_subdirectory(cmake/stm32cubemx)

# 移除错误的 libob.a 依赖（针对 C++）
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# 链接库
target_link_libraries(${CMAKE_PROJECT_NAME}
        stm32cubemx
        m                          # 数学库
        "${DSP_LIB}"               # DSP 库（若未找到则为空）
        # 用户自定义库可追加于此
)

# 后处理：生成 HEX 和 BIN 文件
set(HEX_FILE ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> "${HEX_FILE}"
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> "${BIN_FILE}"
        COMMENT "Generating HEX and BIN files..."
)