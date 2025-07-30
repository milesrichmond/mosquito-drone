# Project Settings
cmake_minimum_required(VERSION 3.23.0)

# CMAKE on MacOS forces these two options. It would otherwise
# assume that using the -arch arm64 compile command is supported
# for god knows what reason.
set(CMAKE_SYSTEM_NAME LINUX)
set(CMAKE_SYSTEM_PROCESSOR ARM)

set(CMAKE_EXECUTABLE_SUFFIX ".elf")
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

project("mosquito-drone")

# TOOLCHAIN
set(TOOLCHAIN_ROOT "arm-none-eabi-")
set(CMAKE_C_COMPILER "${TOOLCHAIN_ROOT}gcc")
set(CMAKE_ASM_COMPILER "${TOOLCHAIN_ROOT}gcc")
set(CMAKE_AR "${TOOLCHAIN_ROOT}ar")
set(CMAKE_LINKER "${TOOLCHAIN_ROOT}ld")
set(CMAKE_OBJCOPY "${TOOLCHAIN_ROOT}objcopy")
set(CMAKE_RANLIB "${TOOLCHAIN_ROOT}ranlib")
set(CMAKE_SIZE "${TOOLCHAIN_ROOT}size")
set(CMAKE_STRIP "${TOOLCHAIN_ROOT}ld")
enable_language(ASM)

# CONFIGURE TO MCU
set(LINKER_SCRIPT "../STM32F103C8TX_FLASH.ld")
set(MCPU "-mcpu=Cortex-M3")
set(MFLOAT_ABI "-mfloat-abi=soft")
set(RUNTIME_LIB "--specs=nano.specs")
set(RUNTIME_LIB_SYS "--specs=rdimon.specs") # enables semihosting

# Build Settings
set(CMAKE_C_FLAGS
    "${MCPU} -std=gnu11 -mthumb ${MFLOAT_ABI} ${RUNTIME_LIB} -Wall -Wpedantic -Werror -u _printf_float -g"
)
set(CMAKE_EXE_LINKER_FLAGS
    "-T${LINKER_SCRIPT} ${RUNTIME_LIB_SYS} -lrdimon -Wl,-Map=test.map -Wl,--gc-sections -static -Wl,--start-group -lc -lm -Wl,--end-group"
)
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp")

# Source Tree
add_executable(${PROJECT_NAME} "${CMAKE_SOURCE_DIR}/Startup/startup_stm32f103c8tx.s")

add_subdirectory(src)

# Include
target_include_directories(${PROJECT_NAME}
    PRIVATE
        include/CMSIS/Include
        include/CMSIS/Device/ST/STM32F1xx/Include
)
