//#include "bno08x.h"
//#include "utils.h"
#include "pico/stdlib.h"
#include "stdio.h"

//i2c_inst_t* i2c_port0;
//initI2C(i2c_port0, false);

// //set up IMU
// while (IMU.begin(CONFIG::BNO08X_ADDR, i2c_port0)==false) {
//     printf("BNO08x not detected at default I2C address. Check wiring. Freezing\n");
//     scan_i2c_bus();
//     sleep_ms(1000);
// }
// IMU.enableRotationVector();

while (true) {

//     float yaw = 0.0f;

//     if (IMU.getSensorEvent() == true) {
//         if (IMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
//             yaw = IMU.getYaw();
//         }
//     }

//     printf("Yaw: %.2f rad\n", yaw);
    printf("Random print\n");
    sleep_ms(1000);

}


return 0;