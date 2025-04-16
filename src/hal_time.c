#include "pico/stdlib.h"

// Provide the current time in milliseconds
uint32_t hal_time_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}