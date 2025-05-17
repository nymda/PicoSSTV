cmake_minimum_required(VERSION 3.13)

set(PICO_SDK_PATH ~/pico/pico-sdk)
set(PICO_EXTRAS_PATH ~/pico/pico-extras)

include(~/pico/pico-sdk/external/pico_sdk_import.cmake)
include(~/pico/pico-extras/external/pico_extras_import.cmake)

project(picoSSTV C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(
	picoSSTV
	src/main_demo.cpp
	src/camera.cpp
	src/hm01b0.c
	src/fontRenderer.cpp
)

target_include_directories(picoSSTV PRIVATE
    .
	src
    $ENV{PICO_SDK_PATH}/lib/tinyusb/hw
    $ENV{PICO_SDK_PATH}/lib/tinyusb/src
)

target_link_libraries(
	picoSSTV
	cmsis_core
	pico_stdlib
	pico_audio_i2s
	pico_multicore
	hardware_i2c
	hardware_pwm
	hardware_flash 
	hardware_sync
)

target_compile_definitions(picoSSTV PRIVATE
	# compile time configuration of I2S
	PICO_AUDIO_I2S_MONO_INPUT=1
	#define for our example code
	USE_AUDIO_I2S=1
)

pico_add_extra_outputs(picoSSTV)

pico_enable_stdio_usb(picoSSTV 1)
pico_enable_stdio_uart(picoSSTV 0)