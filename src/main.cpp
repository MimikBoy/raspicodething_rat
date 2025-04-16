
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "btstack_run_loop.h"
#include "btHandler.h"
#include "vector"
#include "GRF.h"
#include "Angle.h"
#include "bno08x2.h"
#include "utils2.h"
#include "bno08x.h"
#include "utils.h"
#include "hardware/adc.h"

BNO08x IMU;
BNO08x2 IMU2;
float start_time;


int main() {
    stdio_init_all();
    

    // initialize CYW43 driver
    if (cyw43_arch_init()) {
        printf("cyw43_arch_init() failed.\n");
        return -1;
    }
    sleep_ms(3000);
    printf("ojay");
    sleep_ms(3000);
    printf("starting");

    i2c_inst_t* i2c_port0 = i2c0;
    initI2C(i2c_port0, false);
    i2c_inst_t* i2c_port1 = i2c1;
    initI2C2(i2c_port1, false);
    //set up IMU
    while (IMU.begin(CONFIG::BNO08X_ADDR, i2c_port0)==false) {
        printf("BNO08x 1 not detected at default I2C address. Check wiring. Freezing\n");
        scan_i2c_bus();
        sleep_ms(1000);
    }
    while (IMU2.begin(0x4A, i2c_port1)==false) {
        printf("BNO08x 2 not detected at default I2C address. Check wiring. Freezing\n");
        scan_i2c_bus2();
        sleep_ms(1000);
    }

    adc_init();
    adc_gpio_init(26);
    adc_select_input(0);

    IMU.enableStepCounter(2.5);
    IMU.enableLinearAccelerometer(2.5);
    IMU.enableGyro(2.5);
    IMU.enableGameRotationVector(2.5);
    IMU.enableAccelerometer(2.5);
    IMU2.enableAccelerometer(2.5);
    IMU2.enableRotationVector(2.5);

    init_Angle();

    //start_time = time_us_64() / 1000.0f;;

    btstack_main(0, NULL);
    btstack_run_loop_execute();
    
}