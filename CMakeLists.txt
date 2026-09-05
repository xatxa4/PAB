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

# I2S sample queue: 48 kHz max (A2DP/SBC tops out there), 40 ms deep
set(I2S_MAX_FREQ_KHZ 48)
set(I2S_QUEUE_LEN 40)
add_subdirectory(pico_i2s_pio)

add_executable(${PROJECT_NAME})

target_sources(${PROJECT_NAME} PRIVATE
    ${PICO_SDK_PATH}/lib/btstack/src/btstack_audio.c
    src/btstack_audio_pico_i2s.c
    src/main.c
    src/bt.c
    src/sdp.c
    src/a2dp.c
    src/avrcp.c
)

# I2S pinout. Note this differs from the pico-extras driver used previously:
# LRCLK and BCK are swapped, and MCLK is new. A Pimoroni Pico Audio Pack has
# DATA/BCK/LRCLK hard wired to GP9/GP10/GP11 and cannot be used as-is.
target_compile_definitions(${PROJECT_NAME} PRIVATE
    PICO_AUDIO_I2S_DATA_PIN=9         # DIN
    PICO_AUDIO_I2S_CLOCK_PIN_BASE=10  # =LRCLK/WS, +1=BCK
    PICO_AUDIO_I2S_MCLK_PIN=12        # 22.5792 / 24.576 MHz
    # CLOCK_MODE_LOW_JITTER drops the PIO fractional divider by re-running the
    # system PLL. It also re-runs it on every sample rate change, which breaks
    # the CYW43 PIO/SPI bus once Bluetooth is up, so it is off by default.
    PICO_AUDIO_I2S_CLOCK_MODE=CLOCK_MODE_DEFAULT
    PICO_AUDIO_I2S_FORMAT=MODE_I2S
    PICO_AUDIO_I2S_PIO=pio1
    # RUN_PIN=22
    CONN_PIN=26
    BT_PIN="0000"
    BT_NAME="PicoW"
)

target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    pico_stdio_usb  # use usb, not uart for stdio
    # pico_stdio_uart  # use uart, not usb for stdio e.g. via picoprobe
    pico_multicore
    pico-i2s-pio
    pico_btstack_sbc_decoder
    pico_btstack_classic
    pico_btstack_cyw43
    pico_cyw43_arch_none
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR} # For btstack config
)

pico_add_extra_outputs(${PROJECT_NAME})
