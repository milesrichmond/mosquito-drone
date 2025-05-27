# ##############################################################################
# file:  CMakeLists.txt brief: Template "CMakeLists.txt" for building of
# executables and static libraries.
#
# usage: Edit "VARIABLES"-section to suit project requirements. For debug build:
# cmake  -S ./ -B Debug -G"Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug make -C
# Debug VERBOSE=1 -j For release build: cmake -S ./ -B Release -G"Unix
# Makefiles" -DCMAKE_BUILD_TYPE=Release make -C Release VERBOSE=1 -j
# ##############################################################################
cmake_minimum_required(VERSION 3.20)

# ##################### CONSTANTS ######################################
set(PROJECT_TYPE_EXECUTABLE "exe")
set(PROJECT_TYPE_STATIC_LIBRARY "static-lib")
set(MCPU_CORTEX_M0 "-mcpu=cortex-m0")
set(MCPU_CORTEX_M0PLUS "-mcpu=cortex-m0plus")
set(MCPU_CORTEX_M3 "-mcpu=cortex-m3")
set(MCPU_CORTEX_M4 "-mcpu=cortex-m4")
set(MCPU_CORTEX_M7 "-mcpu=cortex-m7")
set(MCPU_CORTEX_M33 "-mcpu=cortex-m33")
set(MCPU_CORTEX_M55 "-mcpu=cortex-m55")
set(MCPU_CORTEX_M85 "-mcpu=cortex-m85")
set(MFPU_FPV4_SP_D16 "-mfpu=fpv4-sp-d16")
set(MFPU_FPV5_D16 "-mfpu=fpv5-d16")
set(RUNTIME_LIBRARY_REDUCED_C "--specs=nano.specs")
set(RUNTIME_LIBRARY_STD_C "")
set(RUNTIME_LIBRARY_SYSCALLS_MINIMAL "--specs=nosys.specs")
set(RUNTIME_LIBRARY_SYSCALLS_NONE "")
set(MFLOAT_ABI_SOFTWARE "-mfloat-abi=soft")
set(MFLOAT_ABI_HARDWARE "-mfloat-abi=hard")
set(MFLOAT_ABI_MIX "-mfloat-abi=softfp")
# ##############################################################################

# ##################### VARIABLES ######################################
set(PROJECT_NAME "mosquito-drone")
set(PROJECT_TYPE "exe")
set(LINKER_SCRIPT "../STM32F103C8TX_FLASH.ld")
set(MCPU "-mcpu=Cortex-M3")
set(MFLOAT_ABI "-mfloat-abi=soft")
set(RUNTIME_LIBRARY "--specs=nano.specs")
# set(RUNTIME_LIBRARY_SYSCALLS "--specs=nosys.specs")
set(RUNTIME_LIBRARY_SYSCALLS "--specs=rdimon.specs")

set(PROJECT_SOURCES # LIST SOURCE FILES HERE
    Startup/startup_stm32f103c8tx.s
    src/main.c
    src/sysmem.c
    src/i2c.c
    src/adafruit-BNO055.c
    src/uart.c
    src/bmp390.c
    src/util.c)

set(PROJECT_DEFINES # LIST COMPILER DEFINITIONS HERE
    DEBUG)

set(PROJECT_INCLUDES # LIST INCLUDE DIRECTORIES HERE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include/CMSIS/Device/ST/STM32F1xx/Include
    ${CMAKE_SOURCE_DIR}/include/CMSIS/Include)

# ########### MODIFY ACCORDING TO REQUIREMENTS) ########################

set(CMAKE_SYSTEM_PROCESSOR
    "arm"
    CACHE STRING "")
set(CMAKE_SYSTEM_NAME
    "Generic"
    CACHE STRING "")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(TOOLCHAIN_PREFIX "arm-none-eabi-")
set(CMAKE_C_COMPILER "${TOOLCHAIN_PREFIX}gcc")
set(CMAKE_ASM_COMPILER "${TOOLCHAIN_PREFIX}gcc")
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_PREFIX}g++")
set(CMAKE_AR "${TOOLCHAIN_PREFIX}ar")
set(CMAKE_LINKER "${TOOLCHAIN_PREFIX}ld")
set(CMAKE_OBJCOPY "${TOOLCHAIN_PREFIX}objcopy")
set(CMAKE_RANLIB "${TOOLCHAIN_PREFIX}ranlib")
set(CMAKE_SIZE "${TOOLCHAIN_PREFIX}size")
set(CMAKE_STRIP "${TOOLCHAIN_PREFIX}ld")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# ##############################################################################

# ################# PROJECT SETUP ######################################
project(${PROJECT_NAME})
enable_language(ASM)

if(${PROJECT_TYPE} MATCHES ${PROJECT_TYPE_EXECUTABLE})
  add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})
  add_custom_command(
    TARGET ${CMAKE_PROJECT_NAME}
    POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>)
elseif(${PROJECT_TYPE} MATCHES ${PROJECT_TYPE_STATIC_LIBRARY})
  add_library(${PROJECT_NAME} ${PROJECT_SOURCES})
endif()

add_compile_definitions(${PROJECT_DEFINES})
include_directories(${PROJECT_INCLUDES})

set(CMAKE_EXECUTABLE_SUFFIX ".elf")
set(CMAKE_STATIC_LIBRARY_SUFFIX ".a")
set(CMAKE_C_FLAGS
    "${MCPU} -std=gnu11 ${MFPU} ${MFLOAT_ABI} ${RUNTIME_LIBRARY} -g -u _printf_float -mthumb -Wall -Werror -Wpedantic"
)
set(CMAKE_EXE_LINKER_FLAGS
    "-T${LINKER_SCRIPT} ${RUNTIME_LIBRARY_SYSCALLS} -g -lrdimon -Wl,-Map=test.map -Wl,--gc-sections -static -Wl,--start-group -lc -lm -Wl,--end-group"
)
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp")
