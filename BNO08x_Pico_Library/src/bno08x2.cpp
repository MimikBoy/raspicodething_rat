/*
  This is a library written for the BNO08x2
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/14686

  Originally written by Nathan Seidle @ SparkFun Electronics, December 28th, 2017

  Adjusted by Pete Lewis @ SparkFun Electronics, June 2023 to incorporate the 
  CEVA Sensor Hub Driver, found here:
  https://github.com/ceva-dsp/sh2

  Also, utilizing code from the Adafruit BNO08x2 Arduino Library by Bryan Siepert 
  for Adafruit Industries. Found here:
  https://github.com/adafruit/Adafruit_BNO08x2

  Also, utilizing I2C and SPI read/write functions and code from the Adafruit 
  BusIO library found here:
  https://github.com/adafruit/Adafruit_BusIO

  Ported by Mark Mellors for use on an RP2040

  The BNO08x2 IMU is a powerful triple axis gyro/accel/magnetometer coupled with an ARM processor
  to maintain and complete all the complex calculations for various VR, inertial, step counting,
  and movement operations.

  This library handles the initialization of the BNO08x2 and is able to query the sensor
  for different readings.

  https://github.com/sparkfun/SparkFun_BNO08x2_Arduino_Library

  SparkFun code, firmware, and software is released under the MIT License.
  Please see LICENSE.md for further details.

  Some of this library was based off of the Adafruit BNO08x2 Arduino Library.
  More specifically, the code layers connecting to the HillCrest/Ceva Driver.
  Their original work can be found here:
  https://github.com/adafruit/Adafruit_BNO08x2
  Thank you Adafruit and your developers for all your hard work put into your Library!
*/

#include "BNO08x2.h"
#include <cmath>
#include <string.h>
#include <algorithm>

#include "utils2.h"
#include "hardware/gpio.h"

int8_t _int_pin_2 = -1, _reset_pin_2 = -1;
static i2c_inst_t  *_i2cPort_2 = NULL;		//The generic connection to user's chosen I2C hardware
static uint8_t _deviceAddress_2 = BNO08x2_DEFAULT_ADDRESS; //Keeps track of I2C address. setI2CAddress changes this.


static sh2_SensorValue_t_2 *_sensor_value = NULL;
static bool _reset_occurred = false;

static int i2chal_write_2(sh2_Hal_t_2 *self, uint8_t *pBuffer, unsigned len);
static int i2chal_read_2(sh2_Hal_t_2 *self, uint8_t *pBuffer, unsigned len,
                       uint32_t *t_us);
static void i2chal_close_2(sh2_Hal_t_2 *self);
static int i2chal_open_2(sh2_Hal_t_2 *self);

static uint32_t hal_getTimeUs_2(sh2_Hal_t_2 *self);
static void hal_callback_2(void *cookie, sh2_AsyncEvent_t_2 *pEvent);
static void sensorHandler_2(void *cookie, sh2_SensorEvent_t_2 *pEvent);

static bool i2c_write_2(const uint8_t *buffer, size_t len, bool stop = true,
        const uint8_t *prefix_buffer = nullptr, size_t prefix_len = 0);

static bool i2c_read_2(uint8_t *buffer, size_t len, bool stop = true);
static bool _i2c_read_2(uint8_t *buffer, size_t len, bool stop);



size_t _maxBufferSize_2 = 32;
size_t maxBufferSize_2();		

//Initializes the sensor with basic settings using I2C
//Returns false if sensor is not detected
bool BNO08x2::begin(uint8_t deviceAddress, i2c_inst_t* i2c_port)
{
  	_deviceAddress_2 = deviceAddress;
  	_i2cPort_2 = i2c_port;




  	if (isConnected() == false) // Check for sensor by verifying ACK response
    	return (false); 

	  printf("I2C address found\n");
    //delay(1000);

    _HAL.open = i2chal_open_2;
    _HAL.close = i2chal_close_2;
    _HAL.read = i2chal_read_2;
    _HAL.write = i2chal_write_2;
    _HAL.getTimeUs = hal_getTimeUs_2;

    return _init();
}


// Quaternion to Euler conversion
// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// https://github.com/sparkfun/SparkFun_MPU-9250-DMP_Arduino_Library/issues/5#issuecomment-306509440
// Return the roll (rotation around the x-axis) in Radians
float BNO08x2::getRoll()
{
	float dqw = getQuatReal();
	float dqx = getQuatI();
	float dqy = getQuatJ();
	float dqz = getQuatK();

	float norm = std::sqrt(dqw*dqw + dqx*dqx + dqy*dqy + dqz*dqz);
	dqw = dqw/norm;
	dqx = dqx/norm;
	dqy = dqy/norm;
	dqz = dqz/norm;

	float ysqr = dqy * dqy;

	// roll (x-axis rotation)
	float t0 = +2.0 * (dqw * dqx + dqy * dqz);
	float t1 = +1.0 - 2.0 * (dqx * dqx + ysqr);
	float roll = std::atan2(t0, t1);

	return (roll);
}

// Return the pitch (rotation around the y-axis) in Radians
float BNO08x2::getPitch()
{
	float dqw = getQuatReal();
	float dqx = getQuatI();
	float dqy = getQuatJ();
	float dqz = getQuatK();

	float norm = std::sqrt(dqw*dqw + dqx*dqx + dqy*dqy + dqz*dqz);
	dqw = dqw/norm;
	dqx = dqx/norm;
	dqy = dqy/norm;
	dqz = dqz/norm;

	//float ysqr = dqy * dqy;

	// pitch (y-axis rotation)
	float t2 = +2.0 * (dqw * dqy - dqz * dqx);
	t2 = t2 > 1.0 ? 1.0 : t2;
	t2 = t2 < -1.0 ? -1.0 : t2;
	float pitch = asin(t2);

	return (pitch);
}

// Return the yaw / heading (rotation around the z-axis) in Radians
float BNO08x2::getYaw()
{
	float dqw = getQuatReal();
	float dqx = getQuatI();
	float dqy = getQuatJ();
	float dqz = getQuatK();

	float norm = std::sqrt(dqw*dqw + dqx*dqx + dqy*dqy + dqz*dqz);
	dqw = dqw/norm;
	dqx = dqx/norm;
	dqy = dqy/norm;
	dqz = dqz/norm;

	float ysqr = dqy * dqy;

	// yaw (z-axis rotation)
	float t3 = +2.0 * (dqw * dqz + dqx * dqy);
	float t4 = +1.0 - 2.0 * (ysqr + dqz * dqz);
	float yaw = std::atan2(t3, t4);

	return (yaw);
}

//Gets the full quaternion
//i,j,k,real output floats
void BNO08x2::getQuat(float &i, float &j, float &k, float &real, float &radAccuracy, uint8_t &accuracy)
{
	i = qToFloat(rawQuatI, rotationVector_Q1);
	j = qToFloat(rawQuatJ, rotationVector_Q1);
	k = qToFloat(rawQuatK, rotationVector_Q1);
	real = qToFloat(rawQuatReal, rotationVector_Q1);
	radAccuracy = qToFloat(rawQuatRadianAccuracy, rotationVector_Q1);
	accuracy = quatAccuracy;
}

//Return the rotation vector quaternion I
float BNO08x2::getQuatI()
{
	// float quat = qToFloat(rawQuatI, rotationVector_Q1);
	// if (_printDebug == true)
	// {
	// 	if ((quat < -1.0) || (quat > 1.0))
	// 	{
	// 		_debugPort->printf("getQuatI: quat: ")); // Debug the occasional non-unitary Quat
	// 		_debugPort->print(quat, 2);
	// 		_debugPort->printf(" rawQuatI: "));
	// 		_debugPort->print(rawQuatI);
	// 		_debugPort->printf(" rotationVector_Q1: "));
	// 		_debugPort->println(rotationVector_Q1);
	// 	}
	// }
	// return (quat);
	return _sensor_value->un.rotationVector.i;
}

//Return the rotation vector quaternion J
float BNO08x2::getQuatJ()
{
	float quat = qToFloat(rawQuatJ, rotationVector_Q1);
	//return (quat);
	return _sensor_value->un.rotationVector.j;
}

//Return the rotation vector quaternion K
float BNO08x2::getQuatK()
{
	float quat = qToFloat(rawQuatK, rotationVector_Q1);
	//return (quat);
	return _sensor_value->un.rotationVector.k;
}

//Return the rotation vector quaternion Real
float BNO08x2::getQuatReal()
{
	return _sensor_value->un.rotationVector.real;
}

//Return the rotation vector radian accuracy
float BNO08x2::getQuatRadianAccuracy()
{
	return _sensor_value->un.rotationVector.accuracy;
}

//Return the rotation vector sensor event report status accuracy
uint8_t BNO08x2::getQuatAccuracy()
{
	return _sensor_value->status;
}

//Return the game rotation vector quaternion I
float BNO08x2::getGameQuatI()
{
	return _sensor_value->un.gameRotationVector.i;
}

//Return the game rotation vector quaternion J
float BNO08x2::getGameQuatJ()
{
	return _sensor_value->un.gameRotationVector.j;
}

//Return the game rotation vector quaternion K
float BNO08x2::getGameQuatK()
{
	return _sensor_value->un.gameRotationVector.k;
}

//Return the game rotation vector quaternion Real
float BNO08x2::getGameQuatReal()
{
	return _sensor_value->un.gameRotationVector.real;
}

//Gets the full acceleration
//x,y,z output floats
void BNO08x2::getAccel(float &x, float &y, float &z, uint8_t &accuracy)
{
	x = qToFloat(rawAccelX, accelerometer_Q1);
	y = qToFloat(rawAccelY, accelerometer_Q1);
	z = qToFloat(rawAccelZ, accelerometer_Q1);
	accuracy = accelAccuracy;
}

//Return the acceleration component
float BNO08x2::getAccelX()
{
	return _sensor_value->un.accelerometer.x;
}

//Return the acceleration component
float BNO08x2::getAccelY()
{
	return _sensor_value->un.accelerometer.y;
}

//Return the acceleration component
float BNO08x2::getAccelZ()
{
	return _sensor_value->un.accelerometer.z;
}

//Return the acceleration component
uint8_t BNO08x2::getAccelAccuracy()
{
	return _sensor_value->status;
}

// linear acceleration, i.e. minus gravity

//Gets the full lin acceleration
//x,y,z output floats
void BNO08x2::getLinAccel(float &x, float &y, float &z, uint8_t &accuracy)
{
	x = qToFloat(rawLinAccelX, linear_accelerometer_Q1);
	y = qToFloat(rawLinAccelY, linear_accelerometer_Q1);
	z = qToFloat(rawLinAccelZ, linear_accelerometer_Q1);
	accuracy = accelLinAccuracy;
}

//Return the acceleration component
float BNO08x2::getLinAccelX()
{
	return _sensor_value->un.linearAcceleration.x;
}

//Return the acceleration component
float BNO08x2::getLinAccelY()
{
	return _sensor_value->un.linearAcceleration.y;
}

//Return the acceleration component
float BNO08x2::getLinAccelZ()
{
	return _sensor_value->un.linearAcceleration.z;
}

//Return the acceleration component
uint8_t BNO08x2::getLinAccelAccuracy()
{
	return _sensor_value->status;
}

//Gets the full gyro vector
//x,y,z output floats
void BNO08x2::getGyro(float &x, float &y, float &z, uint8_t &accuracy)
{
	x = qToFloat(rawGyroX, gyro_Q1);
	y = qToFloat(rawGyroY, gyro_Q1);
	z = qToFloat(rawGyroZ, gyro_Q1);
	accuracy = gyroAccuracy;
}

//Return the gyro component
float BNO08x2::getGyroX()
{
	return _sensor_value->un.gyroscope.x;
}

//Return the gyro component
float BNO08x2::getGyroY()
{
	return _sensor_value->un.gyroscope.y;
}

//Return the gyro component
float BNO08x2::getGyroZ()
{
	return _sensor_value->un.gyroscope.z;
}

//Return the gyro component
uint8_t BNO08x2::getGyroAccuracy()
{
	return (gyroAccuracy);
}

//Gets the full uncalibrated gyro vector
//x,y,z,bx,by,bz output floats
void BNO08x2::getUncalibratedGyro(float &x, float &y, float &z, float &bx, float &by, float &bz, uint8_t &accuracy)
{
	x = _sensor_value->un.gyroscopeUncal.x;
	y = _sensor_value->un.gyroscopeUncal.y;
	z = _sensor_value->un.gyroscopeUncal.z;
	bx = _sensor_value->un.gyroscopeUncal.biasX;
	by = _sensor_value->un.gyroscopeUncal.biasY;
	bz = _sensor_value->un.gyroscopeUncal.biasZ;
	accuracy = _sensor_value->status;
}
//Return the gyro component
float BNO08x2::getUncalibratedGyroX()
{
	return _sensor_value->un.gyroscopeUncal.x;
}
//Return the gyro component
float BNO08x2::getUncalibratedGyroY()
{
	return _sensor_value->un.gyroscopeUncal.y;
}
//Return the gyro component
float BNO08x2::getUncalibratedGyroZ()
{
	return _sensor_value->un.gyroscopeUncal.z;
}
//Return the gyro component
float BNO08x2::getUncalibratedGyroBiasX()
{
	return _sensor_value->un.gyroscopeUncal.biasX;
}
//Return the gyro component
float BNO08x2::getUncalibratedGyroBiasY()
{
	return _sensor_value->un.gyroscopeUncal.biasY;
}
//Return the gyro component
float BNO08x2::getUncalibratedGyroBiasZ()
{
	return _sensor_value->un.gyroscopeUncal.biasZ;
}

//Return the gyro component
uint8_t BNO08x2::getUncalibratedGyroAccuracy()
{
	return (UncalibGyroAccuracy);
}

//Gets the full gravity vector
//x,y,z output floats
void BNO08x2::getGravity(float &x, float &y, float &z, uint8_t &accuracy)
{
	x = qToFloat(gravityX, gravity_Q1);
	y = qToFloat(gravityX, gravity_Q1);
	z = qToFloat(gravityX, gravity_Q1);
	accuracy = gravityAccuracy;
}

float BNO08x2::getGravityX()
{
	return _sensor_value->un.gravity.x;
}

//Return the gravity component
float BNO08x2::getGravityY()
{
	return _sensor_value->un.gravity.y;
}

//Return the gravity component
float BNO08x2::getGravityZ()
{
	return _sensor_value->un.gravity.z;
}

uint8_t BNO08x2::getGravityAccuracy()
{
	return _sensor_value->status;
}

//Gets the full mag vector
//x,y,z output floats
void BNO08x2::getMag(float &x, float &y, float &z, uint8_t &accuracy)
{
	x = qToFloat(rawMagX, magnetometer_Q1);
	y = qToFloat(rawMagY, magnetometer_Q1);
	z = qToFloat(rawMagZ, magnetometer_Q1);
	accuracy = magAccuracy;
}

//Return the magnetometer component
float BNO08x2::getMagX()
{
	return _sensor_value->un.magneticField.x;
}

//Return the magnetometer component
float BNO08x2::getMagY()
{
	return _sensor_value->un.magneticField.y;
}

//Return the magnetometer component
float BNO08x2::getMagZ()
{
	return _sensor_value->un.magneticField.z;
}

//Return the mag component
uint8_t BNO08x2::getMagAccuracy()
{
	return _sensor_value->status;
}

// Return Gyro Integrated Rotation Vector i
float BNO08x2::getGyroIntegratedRVI()
{
	return _sensor_value->un.gyroIntegratedRV.i;
}

// Return Gyro Integrated Rotation Vector j
float BNO08x2::getGyroIntegratedRVJ()
{
	return _sensor_value->un.gyroIntegratedRV.j;
}

// Return Gyro Integrated Rotation Vector k
float BNO08x2::getGyroIntegratedRVK()
{
	return _sensor_value->un.gyroIntegratedRV.k;
}

// Return Gyro Integrated Rotation Vector real
float BNO08x2::getGyroIntegratedRVReal()
{
	return _sensor_value->un.gyroIntegratedRV.real;
}

// Return Gyro Integrated Rotation Vector angVelX
float BNO08x2::getGyroIntegratedRVangVelX()
{
	return _sensor_value->un.gyroIntegratedRV.angVelX;
}

// Return Gyro Integrated Rotation Vector angVelY
float BNO08x2::getGyroIntegratedRVangVelY()
{
	return _sensor_value->un.gyroIntegratedRV.angVelY;
}

// Return Gyro Integrated Rotation Vector angVelZ
float BNO08x2::getGyroIntegratedRVangVelZ()
{
	return _sensor_value->un.gyroIntegratedRV.angVelZ;
}

//Return the tap detector
uint8_t BNO08x2::getTapDetector()
{
	uint8_t previousTapDetector = tapDetector;
	tapDetector = 0; //Reset so user code sees exactly one tap
	return (previousTapDetector);
}

//Return the step count
uint16_t BNO08x2::getStepCount()
{
	return _sensor_value->un.stepCounter.steps;
}

//Return the stability classifier
uint8_t BNO08x2::getStabilityClassifier()
{
	return _sensor_value->un.stabilityClassifier.classification;
}

//Return the activity classifier
uint8_t BNO08x2::getActivityClassifier()
{
	return _sensor_value->un.personalActivityClassifier.mostLikelyState;
}

//Return the activity confindence
uint8_t BNO08x2::getActivityConfidence(uint8_t activity)
{
	return _sensor_value->un.personalActivityClassifier.confidence[activity];
}

//Return the time stamp
uint64_t BNO08x2::getTimeStamp()
{
	return _sensor_value->timestamp;
}

//Return raw mems value for the accel
int16_t BNO08x2::getRawAccelX()
{
	return _sensor_value->un.rawAccelerometer.x;
}
//Return raw mems value for the accel
int16_t BNO08x2::getRawAccelY()
{
	return _sensor_value->un.rawAccelerometer.y;
}
//Return raw mems value for the accel
int16_t BNO08x2::getRawAccelZ()
{
	return _sensor_value->un.rawAccelerometer.z;
}

//Return raw mems value for the gyro
int16_t BNO08x2::getRawGyroX()
{
	return _sensor_value->un.rawGyroscope.x;
}
int16_t BNO08x2::getRawGyroY()
{
	return _sensor_value->un.rawGyroscope.y;
}
int16_t BNO08x2::getRawGyroZ()
{
	return _sensor_value->un.rawGyroscope.z;
}

//Return raw mems value for the mag
int16_t BNO08x2::getRawMagX()
{
	return _sensor_value->un.rawMagnetometer.x;
}
int16_t BNO08x2::getRawMagY()
{
	return _sensor_value->un.rawMagnetometer.y;
}
int16_t BNO08x2::getRawMagZ()
{
	return _sensor_value->un.rawMagnetometer.z;
}

// //Given a record ID, read the Q1 value from the metaData record in the FRS (ya, it's complicated)
// //Q1 is used for all sensor data calculations
// int16_t BNO08x2::getQ1(uint16_t recordID)
// {
// 	//Q1 is always the lower 16 bits of word 7
// 	//uint16_t q = readFRSword(recordID, 7) & 0xFFFF; //Get word 7, lower 16 bits
// 	//return (q);
// }

// //Given a record ID, read the Q2 value from the metaData record in the FRS
// //Q2 is used in sensor bias
// int16_t BNO08x2::getQ2(uint16_t recordID)
// {
// 	//Q2 is always the upper 16 bits of word 7
// 	//uint16_t q = readFRSword(recordID, 7) >> 16; //Get word 7, upper 16 bits
// 	//return (q);
// }

// //Given a record ID, read the Q3 value from the metaData record in the FRS
// //Q3 is used in sensor change sensitivity
// int16_t BNO08x2::getQ3(uint16_t recordID)
// {
// 	//Q3 is always the upper 16 bits of word 8
// 	//uint16_t q = readFRSword(recordID, 8) >> 16; //Get word 8, upper 16 bits
// 	//return (q);
// }

// //Given a record ID, read the resolution value from the metaData record in the FRS for a given sensor
// float BNO08x2::getResolution(uint16_t recordID)
// {
// 	//The resolution Q value are 'the same as those used in the sensor's input report'
// 	//This should be Q1.
// 	int16_t Q = getQ1(recordID);

// 	//Resolution is always word 2
// 	//uint32_t value = readFRSword(recordID, 2); //Get word 2

// 	float resolution = qToFloat(value, Q);

// 	return (resolution);
// }

// //Given a record ID, read the range value from the metaData record in the FRS for a given sensor
// float BNO08x2::getRange(uint16_t recordID)
// {
// 	//The resolution Q value are 'the same as those used in the sensor's input report'
// 	//This should be Q1.
// 	int16_t Q = getQ1(recordID);

// 	//Range is always word 1
// 	//uint32_t value = readFRSword(recordID, 1); //Get word 1

// 	float range = qToFloat(value, Q);

// 	return (range);
// }

bool BNO08x2::serviceBus(void)
{
  sh2_service_2();
  return true;
}

//Send command to reset IC
bool BNO08x2::softReset(void)
{
  int status = sh2_devReset_2();

  if (status != SH2_OK_2) {
    return false;
  }

  return true;	
}

//Set the operating mode to "On"
//(This one is for @jerabaul29)
bool BNO08x2::modeOn(void)
{
  int status = sh2_devOn_2();

  if (status != SH2_OK_2) {
    return false;
  }

  return true;	
}

//Set the operating mode to "Sleep"
//(This one is for @jerabaul29)
bool BNO08x2::modeSleep(void)
{
  int status = sh2_devSleep_2();

  if (status != SH2_OK_2) {
    return false;
  }

  return true;	
}

//Get the reason for the last reset
//1 = POR, 2 = Internal reset, 3 = Watchdog, 4 = External reset, 5 = Other
uint8_t BNO08x2::getResetReason()
{
	return prodIds.entry[0].resetCause;
}

//Given a register value and a Q point, convert to float
//See https://en.wikipedia.org/wiki/Q_(number_format)
float BNO08x2::qToFloat(int16_t fixedPointValue, uint8_t qPoint)
{

	float qFloat = fixedPointValue;
	qFloat *= pow(2, qPoint * -1);
	return (qFloat);
}

//Sends the packet to enable the rotation vector
bool BNO08x2::enableRotationVector(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SH2_ROTATION_VECTOR_2, timeBetweenReports);
}

//Sends the packet to enable the geomagnetic rotation vector
bool BNO08x2::enableGeomagneticRotationVector(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR_2, timeBetweenReports);
}

//Sends the packet to enable the ar/vr stabilized rotation vector
bool BNO08x2::enableARVRStabilizedRotationVector(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR_2, timeBetweenReports);
}

//Sends the packet to enable the rotation vector
bool BNO08x2::enableGameRotationVector(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SH2_GAME_ROTATION_VECTOR_2, timeBetweenReports);
}

//Sends the packet to enable the ar/vr stabilized rotation vector
bool BNO08x2::enableARVRStabilizedGameRotationVector(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR_2, timeBetweenReports);
}

//Sends the packet to enable the accelerometer
bool BNO08x2::enableAccelerometer(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SH2_ACCELEROMETER_2, timeBetweenReports);
}

//Sends the packet to enable the accelerometer
bool BNO08x2::enableLinearAccelerometer(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_LINEAR_ACCELERATION_2, timeBetweenReports);	
}

//Sends the packet to enable the gravity vector
bool BNO08x2::enableGravity(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_GRAVITY_2, timeBetweenReports);	
}

//Sends the packet to enable the gyro
bool BNO08x2::enableGyro(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_GYROSCOPE_CALIBRATED_2, timeBetweenReports);		
}

//Sends the packet to enable the uncalibrated gyro
bool BNO08x2::enableUncalibratedGyro(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_UNCALIBRATED_GYRO_2, timeBetweenReports);		
}

//Sends the packet to enable the magnetometer
bool BNO08x2::enableMagnetometer(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_MAGNETIC_FIELD_2, timeBetweenReports);		
}

//Sends the packet to enable the high refresh-rate gyro-integrated rotation vector
bool BNO08x2::enableGyroIntegratedRotationVector(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR_2, timeBetweenReports);		
}

//Sends the packet to enable the tap detector
bool BNO08x2::enableTapDetector(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_TAP_DETECTOR_2, timeBetweenReports);		
}

//Sends the packet to enable the step counter
bool BNO08x2::enableStepCounter(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_STEP_COUNTER_2, timeBetweenReports);		
}

//Sends the packet to enable the Stability Classifier
bool BNO08x2::enableStabilityClassifier(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_STABILITY_CLASSIFIER_2, timeBetweenReports);		
}

//Sends the packet to enable the raw accel readings
//Note you must enable basic reporting on the sensor as well
bool BNO08x2::enableRawAccelerometer(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_RAW_ACCELEROMETER_2, timeBetweenReports);		
}

//Sends the packet to enable the raw accel readings
//Note you must enable basic reporting on the sensor as well
bool BNO08x2::enableRawGyro(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_RAW_GYROSCOPE_2, timeBetweenReports);		
}

//Sends the packet to enable the raw accel readings
//Note you must enable basic reporting on the sensor as well
bool BNO08x2::enableRawMagnetometer(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_RAW_MAGNETOMETER_2, timeBetweenReports);		
}

//Sends the packet to enable the various activity classifiers
bool BNO08x2::enableActivityClassifier(uint16_t timeBetweenReports, uint32_t activitiesToEnable)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER_2, timeBetweenReports, activitiesToEnable);
}

// See 2.2 of the Calibration Procedure document 1000-4044
// Set the desired sensors to have active dynamic calibration
bool BNO08x2::setCalibrationConfig(uint8_t sensors)
{
  int status = sh2_setCalConfig_2(sensors);

  if (status != SH2_OK_2) {
    return false;
  }

  return true;	
}

bool BNO08x2::tareNow(bool zAxis, sh2_TareBasis_t_2 basis)
{
  int status = sh2_setTareNow_2(zAxis ? TARE_AXIS_Z_2 : TARE_AXIS_ALL_2, basis);

  if (status != SH2_OK_2) {
    return false;
  }

  return true;	
}

bool BNO08x2::saveTare()
{
  int status = sh2_persistTare_2();

  if (status != SH2_OK_2) {
    return false;
  }

  return true;
}

bool BNO08x2::clearTare()
{
  int status = sh2_clearTare_2();

  if (status != SH2_OK_2) {
    return false;
  }

  return true;
}

// //This tells the BNO08x2 to begin calibrating
// //See page 50 of reference manual and the 1000-4044 calibration doc
// void BNO08x2::sendCalibrateCommand(uint8_t thingToCalibrate)
// {
// 	/*shtpData[3] = 0; //P0 - Accel Cal Enable
// 	shtpData[4] = 0; //P1 - Gyro Cal Enable
// 	shtpData[5] = 0; //P2 - Mag Cal Enable
// 	shtpData[6] = 0; //P3 - Subcommand 0x00
// 	shtpData[7] = 0; //P4 - Planar Accel Cal Enable
// 	shtpData[8] = 0; //P5 - Reserved
// 	shtpData[9] = 0; //P6 - Reserved
// 	shtpData[10] = 0; //P7 - Reserved
// 	shtpData[11] = 0; //P8 - Reserved*/

// 	for (uint8_t x = 3; x < 12; x++) //Clear this section of the shtpData array
// 		shtpData[x] = 0;

// 	if (thingToCalibrate == CALIBRATE_ACCEL)
// 		shtpData[3] = 1;
// 	else if (thingToCalibrate == CALIBRATE_GYRO)
// 		shtpData[4] = 1;
// 	else if (thingToCalibrate == CALIBRATE_MAG)
// 		shtpData[5] = 1;
// 	else if (thingToCalibrate == CALIBRATE_PLANAR_ACCEL)
// 		shtpData[7] = 1;
// 	else if (thingToCalibrate == CALIBRATE_ACCEL_GYRO_MAG)
// 	{
// 		shtpData[3] = 1;
// 		shtpData[4] = 1;
// 		shtpData[5] = 1;
// 	}
// 	else if (thingToCalibrate == CALIBRATE_STOP)
// 	{
// 		; //Do nothing, bytes are set to zero
// 	}

// 	//Make the internal calStatus variable non-zero (operation failed) so that user can test while we wait
// 	calibrationStatus = 1;

// 	//Using this shtpData packet, send a command
// 	sendCommand(COMMAND_ME_CALIBRATE);
// }

// //Request ME Calibration Status from BNO08x2
// //See page 51 of reference manual
// void BNO08x2::requestCalibrationStatus()
// {
// 	/*shtpData[3] = 0; //P0 - Reserved
// 	shtpData[4] = 0; //P1 - Reserved
// 	shtpData[5] = 0; //P2 - Reserved
// 	shtpData[6] = 0; //P3 - 0x01 - Subcommand: Get ME Calibration
// 	shtpData[7] = 0; //P4 - Reserved
// 	shtpData[8] = 0; //P5 - Reserved
// 	shtpData[9] = 0; //P6 - Reserved
// 	shtpData[10] = 0; //P7 - Reserved
// 	shtpData[11] = 0; //P8 - Reserved*/

// 	for (uint8_t x = 3; x < 12; x++) //Clear this section of the shtpData array
// 		shtpData[x] = 0;

// 	shtpData[6] = 0x01; //P3 - 0x01 - Subcommand: Get ME Calibration

// 	//Using this shtpData packet, send a command
// 	sendCommand(COMMAND_ME_CALIBRATE);
// }

//This tells the BNO08x2 to save the Dynamic Calibration Data (DCD) to flash
//See page 49 of reference manual and the 1000-4044 calibration doc
bool BNO08x2::saveCalibration()
{
  int status = sh2_saveDcdNow_2();
  if (status != SH2_OK_2) {
    return false;
  }
  return true;	
}

/*!  @brief Initializer for post i2c/spi init
 *   @param sensor_id Optional unique ID for the sensor set
 *   @returns True if chip identified and initialized
 */
bool BNO08x2::_init(int32_t sensor_id) {
  int status;

  hardwareReset();

  // Open SH2 interface (also registers non-sensor event handler.)
  status = sh2_open_2(&_HAL, hal_callback_2, NULL);
  if (status != SH2_OK_2) {
    return false;
  }

  // Check connection partially by getting the product id's
  memset(&prodIds, 0, sizeof(prodIds));
  status = sh2_getProdIds_2(&prodIds);
  if (status != SH2_OK_2) {
    return false;
  }

  // Register sensor listener
  sh2_setSensorCallback_2(sensorHandler_2, NULL);

  return true;
}

/**
 * @brief Check if a reset has occured
 *
 * @return true: a reset has occured false: no reset has occoured
 */
bool BNO08x2::wasReset(void) {
  bool x = _reset_occurred;
  _reset_occurred = false;

  return x;
}

/**
 * @brief Fill the given sensor value object with a new report
 *
 * @param value Pointer to an sh2_SensorValue_t struct to fil
 * @return true: The report object was filled with a new report
 * @return false: No new report available to fill
 */
bool BNO08x2::getSensorEvent() {
  _sensor_value = &sensorValue;

  _sensor_value->timestamp = 0;

  sh2_service_2();

  if (_sensor_value->timestamp == 0 && _sensor_value->sensorId != SH2_GYRO_INTEGRATED_RV_2) {
    // no new events
    return false;
  }

  return true;
}

/**
 * @brief Enable the given report type
 *
 * @param sensorId The report ID to enable
 * @param interval_us The update interval for reports to be generated, in
 * microseconds
 * @param sensorSpecific config settings specific to sensor/reportID.
 * (e.g. enabling/disabling possible activities in personal activity classifier)
 * @return true: success false: failure
 */
bool BNO08x2::enableReport(sh2_SensorId_t_2 sensorId, uint32_t interval_us,
							   	   uint32_t sensorSpecific) {
  static sh2_SensorConfig_t_2 config;

  // These sensor options are disabled or not used in most cases
  config.changeSensitivityEnabled = false;
  config.wakeupEnabled = false;
  config.changeSensitivityRelative = false;
  config.alwaysOnEnabled = false;
  config.changeSensitivity = 0;
  config.batchInterval_us = 0;
  config.sensorSpecific = sensorSpecific;

  config.reportInterval_us = interval_us;
  
  int status = sh2_setSensorConfig_2(sensorId, &config);

  if (status != SH2_OK_2) {
    return false;
  }

  return true;
}

/****************************************
***************************************** I2C interface
*****************************************
*****************************************/

static int i2chal_open_2(sh2_Hal_t_2 *self) {
  // Serial.println("I2C HAL open");

  
  uint8_t softreset_pkt[] = {5, 0, 1, 0, 1};
  bool success = false;
  for (uint8_t attempts = 0; attempts < 5; attempts++) {
    if (i2c_write_2(softreset_pkt, 5)) {
      success = true;
      break;
    }
    sleep_ms(30);
  }
  if (!success)
    return -1;
  sleep_ms(300);
  return 0;
}

static void i2chal_close_2(sh2_Hal_t_2 *self) {
  // Serial.println("I2C HAL close");
}

static int i2chal_read_2(sh2_Hal_t_2 *self, uint8_t *pBuffer, unsigned len,
                       uint32_t *t_us) {
  // Serial.println("I2C HAL read");

  // uint8_t *pBufferOrig = pBuffer;

  uint8_t header[4];
  if (!i2c_read_2(header, 4)) {
    return 0;
  }

  // Determine amount to read
  uint16_t packet_size = (uint16_t)header[0] | (uint16_t)header[1] << 8;
  // Unset the "continue" bit
  packet_size &= ~0x8000;

  /*
  Serial.print("Read SHTP header. ");
  Serial.print("Packet size: ");
  Serial.print(packet_size);
  Serial.print(" & buffer size: ");
  Serial.println(len);
  */

  size_t i2c_buffer_max = maxBufferSize_2();

  if (packet_size > len) {
    // packet wouldn't fit in our buffer
    return 0;
  }
  // the number of non-header bytes to read
  uint16_t cargo_remaining = packet_size;
  uint8_t i2c_buffer[i2c_buffer_max];
  uint16_t read_size;
  uint16_t cargo_read_amount = 0;
  bool first_read = true;

  while (cargo_remaining > 0) {
    if (first_read) {
      read_size = std::min(i2c_buffer_max, (size_t)cargo_remaining);
    } else {
      read_size = std::min(i2c_buffer_max, (size_t)cargo_remaining + 4);
    }

    // Serial.print("Reading from I2C: "); Serial.println(read_size);
    // Serial.print("Remaining to read: "); Serial.println(cargo_remaining);


    if (!i2c_read_2(i2c_buffer, read_size)) {
      return 0;
    }

    if (first_read) {
      // The first time we're saving the "original" header, so include it in the
      // cargo count
      cargo_read_amount = read_size;
      memcpy(pBuffer, i2c_buffer, cargo_read_amount);
      first_read = false;
    } else {
      // this is not the first read, so copy from 4 bytes after the beginning of
      // the i2c buffer to skip the header included with every new i2c read and
      // don't include the header in the amount of cargo read
      cargo_read_amount = read_size - 4;
      memcpy(pBuffer, i2c_buffer + 4, cargo_read_amount);
    }
    // advance our pointer by the amount of cargo read
    pBuffer += cargo_read_amount;
    // mark the cargo as received
    cargo_remaining -= cargo_read_amount;
  }

  /*
  for (int i=0; i<packet_size; i++) {
    Serial.print(pBufferOrig[i], HEX);
    Serial.print(", ");
    if (i % 16 == 15) Serial.println();
  }
  Serial.println();
  */

  return packet_size;
}

static int i2chal_write_2(sh2_Hal_t_2 *self, uint8_t *pBuffer, unsigned len) {
  size_t i2c_buffer_max = maxBufferSize_2();

  /*
  Serial.print("I2C HAL write packet size: ");
  Serial.print(len);
  Serial.print(" & max buffer size: ");
  Serial.println(i2c_buffer_max);
  */

  uint16_t write_size = std::min(i2c_buffer_max, len);

  if (!i2c_write_2(pBuffer, write_size)) {
    return 0;
  }

  return write_size;
}

/****************************************
***************************************** HAL interface
*****************************************
*****************************************/

static void hal_hardwareReset(void) {
  if (_reset_pin_2 != -1) {
    // Serial.println("BNO08x2 Hardware reset");

    gpio_init(_reset_pin_2);
    gpio_set_dir(_reset_pin_2, GPIO_OUT);
    gpio_put(_reset_pin_2, 1);
    sleep_ms(10);
    gpio_put(_reset_pin_2, 0);
    sleep_ms(10);
    gpio_put(_reset_pin_2, 1);
    sleep_ms(10);
  }
}

static uint32_t hal_getTimeUs_2(sh2_Hal_t_2 *self) {
  uint64_t t = to_us_since_boot(get_absolute_time());
  // Serial.printf("I2C HAL get time: %d\n", t);
  return t;
}

static void hal_callback_2(void *cookie, sh2_AsyncEvent_t_2 *pEvent) {
  // If we see a reset, set a flag so that sensors will be reconfigured.
  if (pEvent->eventId == SH2_RESET_2) {
    // Serial.println("Reset!");
    _reset_occurred = true;
  }
}

// Handle sensor events.
static void sensorHandler_2(void *cookie, sh2_SensorEvent_t_2 *event) {
  int rc;

  // Serial.println("Got an event!");

  rc = sh2_decodeSensorEvent_2(_sensor_value, event);
  if (rc != SH2_OK_2) {
    //Serial.println("BNO08x2 - Error decoding sensor event");
    _sensor_value->timestamp = 0;
    return;
  }
}

/**
 * @brief Reset the device using the Reset pin
 *
 */
void BNO08x2::hardwareReset(void) { hal_hardwareReset(); }


//Return the sensorID
uint8_t BNO08x2::getSensorEventID()
{
	return _sensor_value->sensorId;
}


//Returns true if I2C device ack's
bool BNO08x2::isConnected()
{
    uint8_t dummy;
    // Attempt to read a single byte from the device
    const int result = i2c_read_timeout_us(_i2cPort_2, _deviceAddress_2, &dummy, 1, false, CONFIG_2::I2C_TIMEOUT_US);

	  if (result == PICO_ERROR_GENERIC || result == PICO_ERROR_TIMEOUT) {
		  // re-init the i2c port
		  handleI2CError2(_i2cPort_2);
	  }

    // If the result is positive, the read was successful, which means the device is connected
    return (result > 0);
}

/****************************************
***************************************** I2C Write/Read Functions
*****************************************
*****************************************/

/*!
 *    @brief  Write a buffer or two to the I2C device. Cannot be more than
 * maxBufferSize_2() bytes.
 *    @param  buffer Pointer to buffer of data to write. This is const to
 *            ensure the content of this buffer doesn't change.
 *    @param  len Number of bytes from buffer to write
 *    @param  prefix_buffer Pointer to optional array of data to write before
 * buffer. Cannot be more than maxBufferSize_2() bytes. This is const to
 *            ensure the content of this buffer doesn't change.
 *    @param  prefix_len Number of bytes from prefix buffer to write
 *    @param  stop Whether to send an I2C STOP signal on write
 *    @return True if write was successful, otherwise false.
 */
bool i2c_write_2(const uint8_t *buffer, size_t len, bool stop,
                               const uint8_t *prefix_buffer,
                               size_t prefix_len) {
    // Check buffer size - Adjust as per your platform's capabilities
    if ((len + prefix_len) > maxBufferSize_2()) {
        return false;
    }

    // Buffer to hold prefix and data combined
    uint8_t combined_buffer[maxBufferSize_2()];
    size_t total_len = 0;

    // Add prefix data if present
    if (prefix_len != 0 && prefix_buffer != nullptr) {
        memcpy(combined_buffer, prefix_buffer, prefix_len);
        total_len += prefix_len;
    }

    // Add main data
    memcpy(combined_buffer + total_len, buffer, len);
    total_len += len;

    // Perform the I2C write
    int bytes_written = i2c_write_timeout_us(_i2cPort_2, _deviceAddress_2, combined_buffer, total_len, !stop, CONFIG_2::I2C_TIMEOUT_US);
    
    if (bytes_written == PICO_ERROR_GENERIC || bytes_written == PICO_ERROR_TIMEOUT) {
		    // re-init the i2c port
		    handleI2CError2(_i2cPort_2);
	  }

    // Check if all bytes were written
    return bytes_written == total_len;
}

/*!
 *    @brief  Read from I2C into a buffer from the I2C device.
 *    Cannot be more than maxBufferSize_2() bytes.
 *    @param  buffer Pointer to buffer of data to read into
 *    @param  len Number of bytes from buffer to read.
 *    @param  stop Whether to send an I2C STOP signal on read
 *    @return True if read was successful, otherwise false.
 */
bool i2c_read_2(uint8_t *buffer, size_t len, bool stop) {
  size_t pos = 0;
  while (pos < len) {
    size_t read_len =
        ((len - pos) > maxBufferSize_2()) ? maxBufferSize_2() : (len - pos);
    bool read_stop = (pos < (len - read_len)) ? false : stop;
    if (!_i2c_read_2(buffer + pos, read_len, read_stop))
      return false;
    pos += read_len;
  }
  return true;
}

bool _i2c_read_2(uint8_t *buffer, size_t len, bool stop) {
    // Perform the I2C read
    int bytes_read = i2c_read_timeout_us(_i2cPort_2, _deviceAddress_2, buffer, len, !stop, CONFIG_2::I2C_TIMEOUT_US);

	  if (bytes_read == PICO_ERROR_GENERIC || bytes_read == PICO_ERROR_TIMEOUT) {
		  // re-init the i2c port
		  handleI2CError2(_i2cPort_2);
	  }

    // Check if the number of bytes read is as expected
    return bytes_read == len;
}

  /*!   @brief  How many bytes we can read in a transaction
   *    @return The size of the Wire receive/transmit buffer */
size_t maxBufferSize_2() { return _maxBufferSize_2; }



