cmake_minimum_required(VERSION 3.13)
 
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
 
# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
 
set(PICO_BOARD pico CACHE STRING "Board type")
 
include(pico_sdk_import.cmake)
 
project(core_o_echo C CXX ASM)
 
pico_sdk_init()
 
# ============================================================
# PRODUCCIÓN: Core 0 + Core 1 completos
# ============================================================
add_executable(core_o_echo
    core0_main.c
    core1_ai.c
    mpu6050.c
    mlp_weights.c
)
 
target_include_directories(core_o_echo PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)
 
pico_generate_pio_header(core_o_echo ${CMAKE_CURRENT_LIST_DIR}/mux_adc.pio)
 
target_link_libraries(core_o_echo
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_dma
    hardware_adc
    hardware_i2c
    hardware_uart
    hardware_gpio
    hardware_clocks
    hardware_timer
    m                  # math.h — requerido para expf() en softmax del Core 1
)
 
pico_set_program_name(core_o_echo "core_o_echo")
pico_set_program_version(core_o_echo "1.0")
 
pico_enable_stdio_usb(core_o_echo 1)
pico_enable_stdio_uart(core_o_echo 0)
pico_add_extra_outputs(core_o_echo)