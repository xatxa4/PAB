cmake_minimum_required(VERSION 3.5)

# This FORCES all background sub-projects (like elf2uf2) to bypass the error
set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING "" FORCE)
set(ENV{CMAKE_POLICY_VERSION_MINIMUM} 3.5)


if (NOT DEFINED ENV{PICO_SDK_PATH} AND (NOT PICO_SDK_PATH))
    set(PICO_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/../pico-sdk)
endif ()

if (NOT DEFINED ENV{PICO_BOARD} AND (NOT PICO_BOARD))
    set(PICO_BOARD pico_w)
endif ()

include(pico_sdk_import.cmake)

pico_sdk_init()

project(picow-a2dp CXX C ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_executable(${PROJECT_NAME})

# only the PIO programs are used from pico_i2s_pio, not its queue/DMA layer
pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/pico_i2s_pio/i2s.pio)

set(PAB_SOURCES
    src/audio_out.c
    src/app_mode.c
    src/btstack_audio_pico_i2s.c
    src/mode_button.c
    src/usb_dac.c
    src/main.c
    src/bt.c
    src/sdp.c
    src/a2dp.c
    src/avrcp.c
)

target_sources(${PROJECT_NAME} PRIVATE
    ${PICO_SDK_PATH}/lib/btstack/src/btstack_audio.c
    ${PAB_SOURCES}
)

# Warnings for our own files only. The SDK's libraries are INTERFACE libraries
# that compile into this target too, and their warnings are not ours to fix.
set_source_files_properties(${PAB_SOURCES} PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")

# Stamp the binary with the tree it was built from. Runs on every build, not
# just at configure time, and rewrites the header only when the result changes,
# so it never goes stale and never forces a rebuild by itself.
set(PAB_GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
add_custom_target(pab_version
    COMMAND ${CMAKE_COMMAND}
        -DSOURCE_DIR=${CMAKE_CURRENT_LIST_DIR}
        -DOUTPUT=${PAB_GENERATED_DIR}/pab_version.h
        -P ${CMAKE_CURRENT_LIST_DIR}/cmake/pab_version.cmake
    BYPRODUCTS ${PAB_GENERATED_DIR}/pab_version.h
    COMMENT "Stamping git version"
    VERBATIM
)
add_dependencies(${PROJECT_NAME} pab_version)

# Same wiring as usb_sound_card_hires: i2s_mclk_set_pin(18, 20, 22)
target_compile_definitions(${PROJECT_NAME} PRIVATE
    PICO_AUDIO_I2S_DATA_PIN=18        # DIN
    PICO_AUDIO_I2S_CLOCK_PIN_BASE=20  # =LRCLK/WS, +1=BCK
    # MCLK off: the ES9038Q2M has its own clock, and a 22.5MHz output next to the
    # CYW43 SPI pins is the one electrical difference from joba-1's working build.
    # Set to 22 to drive it on GP22 again.
    # PICO_AUDIO_I2S_MCLK_PIN=22
    PICO_AUDIO_I2S_PIO=pio1
    # ~46ms of audio queued ahead of the DAC. Lower for less delay. Sized in
    # time, so it means the same at every rate up to the maximum.
    PICO_AUDIO_I2S_NUM_BUFFERS=4
    PICO_AUDIO_I2S_BUFFER_US=11610    # 512 frames at 44.1kHz
    PICO_AUDIO_I2S_MAX_SAMPLE_RATE=96000
    # RUN_PIN=22
    CONN_PIN=26
    BT_PIN="0000"
    BT_NAME="PicoW"
)

# Debug output: usb | uart | none.
#   usb   development default. Costs a periodic TinyUSB task on core0 forever,
#         and occupies the USB peripheral.
#   uart  frees USB, e.g. for a future UAC sound-card mode. Needs a probe.
#   none  lowest jitter, no console. What a finished build should ship with.
if (NOT DEFINED PAB_STDIO)
    set(PAB_STDIO usb)
endif()

target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    hardware_clocks
    hardware_dma
    hardware_flash
    hardware_pio
    pico_flash
    pico_btstack_sbc_decoder
    pico_btstack_classic
    pico_btstack_cyw43
    pico_cyw43_arch_none
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR} # For btstack config
    ${PAB_GENERATED_DIR}      # pab_version.h
)

# Both drivers set explicitly: pico_stdlib enables stdio UART for every target
# unless told otherwise, so linking only the one we want left UART on in all
# three builds - including "none".
if (PAB_STDIO STREQUAL "usb")
    pico_enable_stdio_usb(${PROJECT_NAME} 1)
    pico_enable_stdio_uart(${PROJECT_NAME} 0)
elseif (PAB_STDIO STREQUAL "uart")
    pico_enable_stdio_usb(${PROJECT_NAME} 0)
    pico_enable_stdio_uart(${PROJECT_NAME} 1)
elseif (PAB_STDIO STREQUAL "none")
    pico_enable_stdio_usb(${PROJECT_NAME} 0)
    pico_enable_stdio_uart(${PROJECT_NAME} 0)
else()
    message(FATAL_ERROR "PAB_STDIO must be usb, uart or none (got '${PAB_STDIO}')")
endif()
message(STATUS "PAB stdio: ${PAB_STDIO}")

pico_add_extra_outputs(${PROJECT_NAME})
