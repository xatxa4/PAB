#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/watchdog.h"

#include "app_mode.h"
#include "bt.h"
#include "usb_dac.h"


// Unrecoverable error happened. Reboot by setting watchdog.
// Blink led until watchdog fires
// If RUN_PIN is defined then try reset via run pin after 5 blinks
void fatal() {
    watchdog_enable(1000, true);  // reboot in 1s
    #ifdef RUN_PIN
        unsigned count = 0;
    #endif
    while(true) {  // blink until reboot
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);
        sleep_ms(20);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);
        sleep_ms(80);
        #ifdef RUN_PIN
            if (++count >= 5) {
                // Pull run pin low, to reset the pico
                gpio_init(RUN_PIN);
                gpio_set_dir(RUN_PIN, GPIO_OUT);
                gpio_put(RUN_PIN, (count & 1) ? false : true);
            }
        #endif
    }
}


void on_bt_up( void * ) {
    printf("Bluetooth stack is up\n");
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);
}


static void bt_sink_run(void) {
    // initialize CYW43 driver architecture (will enable BT if/because CYW43_ENABLE_BLUETOOTH == 1)
    if (cyw43_arch_init()) {
        printf("Failed to init cyw43_arch\n");
        fatal();
    }

    // led on during setup until bt is up
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);

    bt_begin(BT_NAME, BT_PIN, on_bt_up, NULL);

    printf("Setup done\n");
    bt_run();
}


int main() {
    stdio_init_all();

    app_mode_t mode = app_mode_current();
    printf("MODE            : %s\n", app_mode_name(mode));

    switch (mode) {
        case APP_MODE_USB_DAC:
            usb_dac_run();
            break;
        case APP_MODE_BT_SINK:
        default:
            bt_sink_run();
            break;
    }

    fatal();
    return -2;
}
