cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
set(PICO_SDK_PATH "$ENV{HOME}/servo2040/pico-sdk")
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Set the path to pimoroni-pico
set(PIMORONI_PICO_PATH "$ENV{HOME}/servo2040/pimoroni-pico")
include(${PIMORONI_PICO_PATH}/pimoroni_pico_import.cmake)
# Include the button driver
include(${PIMORONI_PICO_PATH}/drivers/button/button.cmake)

project(servo2040_controller C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

# Add only the main servo2040 library (it includes everything else)
add_subdirectory(${PIMORONI_PICO_PATH}/libraries/servo2040 servo2040_lib)
# No need to add button library separately as it's part of servo2040

# Add your source files
add_executable(servo2040_controller
    servo2040_controller.cpp
    ${PIMORONI_PICO_PATH}/drivers/button/button.cpp
)

# Include the correct path for the button.hpp file
target_include_directories(servo2040_controller PRIVATE
    ${PIMORONI_PICO_PATH}/drivers
)

# Remove manual PIO generation since servo2040 library handles this
# pico_generate_pio_header(servo2040_controller ${PIMORONI_PICO_PATH}/drivers/plasma/ws2812.pio)
# pico_generate_pio_header(servo2040_controller ${PIMORONI_PICO_PATH}/drivers/pwm/pwm_cluster.pio)

# Include required libraries - simplified since servo2040 includes everything
target_link_libraries(servo2040_controller
    pico_stdlib
    servo2040          # This includes servo, plasma, rgbled, button, etc.
    hardware_pwm       # PWM support
    hardware_clocks    # Clock support
    hardware_adc       # ADC for sensors
    hardware_pio       # PIO support
    hardware_dma       # DMA support
)

# Include directories are handled by the servo2040 library
# target_include_directories handled automatically

# Enable USB output, disable UART output
pico_enable_stdio_usb(servo2040_controller 1)
pico_enable_stdio_uart(servo2040_controller 0)

# Create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(servo2040_controller)