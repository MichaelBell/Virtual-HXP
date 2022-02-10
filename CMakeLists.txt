# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_COPY_TO_RAM 1)

# Pull in Pico SDK (must be before project)
include(pico_sdk_import.cmake)
include(pico_extras_import.cmake)

project(VirtualHXP C CXX ASM)

# Initialise the Pico SDK
pico_sdk_init()

add_subdirectory(i2c_slave)
add_subdirectory(PicoDVI/software/libdvi)
include_directories(PicoDVI/software/include PicoDVI/software/assets RDM)

# Add executable. Default name is the project name, version 0.1

add_executable(VirtualHXP VirtualHXP.cpp VHXPData.cpp VHXPServoInput.c terminal.c tmds_encode_font_2bpp.S)

target_compile_definitions(VirtualHXP PRIVATE 
  DVI_VERTICAL_REPEAT=1
  DVI_DEFAULT_SERIAL_CONFIG=pico_sock_cfg
  PICO_CORE1_STACK_SIZE=0x200
  )

pico_set_program_name(VirtualHXP "VirtualHXP")
pico_set_program_version(VirtualHXP "0.1")

pico_enable_stdio_uart(VirtualHXP 1)
pico_enable_stdio_usb(VirtualHXP 0)

# Add the standard library to the build
target_link_libraries(VirtualHXP pico_stdlib pico_multicore)

# Add any user requested libraries
target_link_libraries(VirtualHXP
		i2c_slave
    libdvi
        )

pico_add_extra_outputs(VirtualHXP)

