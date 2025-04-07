#include "bno08x.h"

int main() {
    // Set up I2C interface
    i2c_inst_t* i2c_port0;
    initI2C(i2c_port0, false);

    // Initialize sensor: attempt to start the IMU and wait until it's detected.
    while (IMU.begin(CONFIG::BNO08X_ADDR, i2c_port0) == false) {
        printf("BNO08x not detected at default I2C address. Check wiring. Freezing\n");
        scan_i2c_bus();
        sleep_ms(1000);
    }

    // Enable the rotation vector sensor
    IMU.enableRotationVector();

    // Main loop: read and print the yaw angle continuously.
    while (true) {
        float yaw = 0.0f;
        if (IMU.getSensorEvent() == true) {
            if (IMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
                yaw = IMU.getYaw();
            }
        }
        printf("Yaw: %.2f rad\n", yaw);
    }

    return 0;
}