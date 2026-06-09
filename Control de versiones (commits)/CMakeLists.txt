# Generated Cmake Pico project file
 
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
 
# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)
 
project(core_o_echo C CXX ASM)
 
# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()
 
# Ejecutable de producción Core 0
add_executable(core_o_echo
    core0_main.c
    mpu6050.c
)
 
# Generar encabezado del programa PIO
pico_generate_pio_header(core_o_echo ${CMAKE_CURRENT_LIST_DIR}/mux_adc.pio)
 
pico_set_program_name(core_o_echo "core_o_echo")
pico_set_program_version(core_o_echo "0.1")
 
# Salida por USB (monitor serial y debug)
pico_enable_stdio_uart(core_o_echo 0)
pico_enable_stdio_usb(core_o_echo 1)
 
# Librerías de hardware requeridas
target_link_libraries(core_o_echo
    pico_stdlib
    pico_multicore        # Comunicación y arranque del Core 1
    hardware_pio          # Máquinas de estado PIO para el MUX
    hardware_dma          # Transporte autónomo de datos ADC → SRAM
    hardware_adc          # Digitalización analógica en GP26
    hardware_i2c          # Comunicación con IMU MPU-6050
    hardware_uart         # DFPlayer Mini y monitor PC
    hardware_gpio         # Control de pines y funciones GPIO
    hardware_clocks       # Configuración del divisor de reloj del PIO
    hardware_timer        # Temporizador periódico a 50Hz
)
 
# Directorio de includes
target_include_directories(core_o_echo PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)
 
pico_add_extra_outputs(core_o_echo)