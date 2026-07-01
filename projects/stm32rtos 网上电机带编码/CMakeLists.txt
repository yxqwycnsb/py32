cmake_minimum_required(VERSION 3.13)
project(stm32rtos C ASM)

set(MCU cortex-m3)
set(MCPU_FLAGS -mcpu=${MCU} -mthumb)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_FLAGS "${MCPU_FLAGS} -Os -g -ffunction-sections -fdata-sections")
set(CMAKE_ASM_FLAGS "${MCPU_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${MCPU_FLAGS} -Wl,--gc-sections -T ${LINKER_SCRIPT}")

# Toolchain variables (used if not provided by command line)
if(NOT DEFINED CMAKE_OBJCOPY)
  find_program(CMAKE_OBJCOPY arm-none-eabi-objcopy)
endif()

# Collect sources (adjust patterns if needed)
file(GLOB_RECURSE SRC_FILES
    "User/*.c"
    "hardwork/*.c"
    "Library/*.c"
    "system/*.c"
    "freertos/*.c"
    "Start/*.c"
    "*.c"
)

add_executable(firmware ${SRC_FILES})

# Size and binary outputs
add_custom_command(TARGET firmware POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:firmware> ${CMAKE_BINARY_DIR}\\firmware.bin
  COMMAND ${CMAKE_SIZE} $<TARGET_FILE:firmware>
)

message(STATUS "Project configured. Use: cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=arm-none-eabi.toolchain.cmake -DLINKER_SCRIPT=Start/Project.sct")