#include <Math.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define DEBUG_PRINT 0
const double DEG_PER_RAD = 180.0 / M_PI;
/*
 * Orientation Sensor Test
 *
 * ------------------------------------
 * Sensor:     BNO055
 * Driver Ver: 1
 * Unique ID:  55
 * Max Value:  0.00 xxx
 * Min Value:  0.00 xxx
 * Resolution: 0.01 xxx
 * ------------------------------------
 *
 * i:245.25,-0.13,0.94,0.00,0.00,0.00,0.00,0.00,0.00,0,0,0,0,3,3,0,3,21,143493,8,
 * i:245.25,-0.13,0.94,0.00,0.00,0.00,0.00,0.00,0.00,0,0,0,0,3,3,0,3,21,143603,792,
 * i:245.25,-0.13,0.94,0.00,0.00,0.00,0.00,0.00,0.00,0,0,0,0,3,3,0,3,21,143714,480,
 */

class InertialController {

public:

InertialController(void) {
    _bn055 = new Adafruit_BNO055(55);
    /* Initialize the sensor */
    if(!_bn055->begin()) {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("No BNO055 detected!");
        while(1);
    }
    /* Use external crystal for better accuracy */
    _bn055->setExtCrystalUse(false);
}

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
/* Display some basic information on this sensor */
void displaySensorDetails(void) {
    sensor_t sensor;
    _bn055->getSensor(&sensor);

#if DEBUG_PRINT
    Serial.print("------------------------------------\nSensor:     ");
    Serial.print(sensor.name);
    Serial.print("\nDriver Ver: ");
    Serial.print(sensor.version);
    Serial.print("\nUnique ID:  ");
    Serial.print(sensor.sensor_id);
    Serial.print("\nMax Value:  ");
    Serial.print(sensor.max_value);
    Serial.print(" xxx\nMin Value:  ");
    Serial.print(sensor.min_value);
    Serial.print(" xxx\nResolution: ");
    Serial.print(sensor.resolution);
    Serial.print(" xxx\n------------------------------------\n");
    delay(500);
#endif

}

/** Board layout:
 *     +----------+
 *     |         *| RST   PITCH  ROLL  HEADING
 * ADR |*        *| SCL
 * INT |*        *| SDA     ^            /->
 * PS1 |*        *| GND     |            |
 * PS0 |*        *| 3VO     Y    Z-->    \-X
 *     |         *| VIN
 *     +----------+
 */

void pollSensorData(void) {
    /* Get a new sensor event */
    sensors_event_t event;
    _bn055->getEvent(&event);
    // Orientation:
    _heading = (float)event.orientation.x;
    _pitch = (float)event.orientation.y;
    _roll = (float)event.orientation.z;
    // Calibration:
    /* Also get calibration data for each sensor. */
    _bn055->getCalibration(&_calibrationSys, &_calibrationGyro
            , &_calibrationAccel, &_calibrationMag);

    // TODO: make user indicators for each calibration status.
    if(_calibrationMag != 0) {
        digitalWrite(A2, HIGH);
    } else {
        digitalWrite(A2, LOW);
    }

    /* Get a new sensor event */
    imu::Vector<3> vec = _bn055->getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    // - VECTOR_LINEARACCEL   - m/s^2
    double accelX = vec.x() - OFFSET_ACCEL_X;
    double accelY = vec.y() - OFFSET_ACCEL_Y;
    double accelZ = vec.z() - OFFSET_ACCEL_Z;

    vec = _bn055->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    double gyroX = vec.x();
    double gyroY = vec.y();
    double gyroZ = vec.z();

#if DEBUG_PRINT
    Serial.print("accX: ");
    Serial.print(accelX);
    Serial.print(", accelY: ");
    Serial.print(accelY);
    Serial.print(", accelZ: ");
    Serial.print(accelZ);
    Serial.print("\n");
    Serial.print("gyroX: ");
    Serial.print(gyroX);
    Serial.print(", gyroY: ");
    Serial.print(gyroY);
    Serial.print(", gyroZ: ");
    Serial.print(gyroZ);
    Serial.print("\n");
#endif

    // If any of these are non zero, then we are in motion.
    // Maybe round to the nearest hundredth or thousanth.
    if(abs(accelX) > 0.2 || abs(accelY) > 0.2 || abs(accelZ) > 0.3
            || abs(gyroX) > 0.05 || abs(gyroY) > 0.05 || abs(gyroZ) > 0.05) {

        double seconds = (double)_deltaTime / 1000000.0;

        if(_calibrationAccel != 0 || _calibrationSys !=0) {
            _accSpeedX += accelX * seconds;
            _accSpeedY += accelY * seconds;
            _accSpeedZ += accelZ * seconds;
            //
            _accDistX += _accSpeedX * seconds;
            _accDistY += _accSpeedY * seconds;
            _accDistZ += _accSpeedZ * seconds;
        }

        if(_calibrationGyro != 0) {
            _gyroDegreesX += gyroX * DEG_PER_RAD * seconds;
            _gyroDegreesY += gyroY * DEG_PER_RAD * seconds;
            _gyroDegreesZ += gyroZ * DEG_PER_RAD * seconds;
        }
        _inMotion = true;
    } else {
        // Decay the speed derived from linear acceleration.
        _accSpeedX *= 0.9;
        _accSpeedY *= 0.9;
        _accSpeedZ *= 0.9;
        _inMotion = false;
    }

    // Current Temperature in C:
    /* Get the current temperature */
    _temperature = _bn055->getTemp();
}

void setTimestamp(unsigned long milliseconds, unsigned short microseconds) {

    _prevMilliseconds = _milliseconds; // Store the previous time-stamp.
    _prevMicroseconds = _microseconds;
    _milliseconds = milliseconds; // Store the current time-stamp.
    _microseconds = microseconds;
    // Now we have delta T in microseconds.
    _deltaTime = ((_milliseconds * 1000) + _microseconds)
        - ((_prevMilliseconds * 1000) + _prevMicroseconds);
}

unsigned long getMilliseconds(void) {
    return _milliseconds;
}

unsigned short getMicroseconds(void) {
    return _microseconds;
}

float getHeading(void) {
    return _heading;
}

float getPitch(void) {
    return _pitch;
}

float getRoll(void) {
    return _roll;
}

// Meters per second
float getAccSpeedX(void) {
    return _accSpeedX;
}

float getAccSpeedY(void) {
    return _accSpeedY;
}

float getAccSpeedZ(void) {
    return _accSpeedZ;
}

float getAccDistX(void) {
    return _accDistX;
}

float getAccDistY(void) {
    return _accDistY;
}

float getAccDistZ(void) {
    return _accDistZ;
}

// Maybe long int is needed
int getGyroDegreesX(void) {
    return _gyroDegreesX;
}

int getGyroDegreesY(void) {
    return _gyroDegreesY;
}

int getGyroDegreesZ(void) {
    return _gyroDegreesZ;
}

bool getInMotion(void) {
    return _inMotion;
}

unsigned char getCalibrationSys(void) {
    return _calibrationSys;
}

unsigned char getCalibrationGyro(void) {
    return _calibrationGyro;
}

unsigned char getCalibrationAccel(void) {
    return _calibrationAccel;
}

unsigned char getCalibrationMag(void) {
    return _calibrationMag;
}

int8_t getTemperature(void) {
    return _temperature;
}

unsigned int getDeltaMilliseconds(void) {
    return (unsigned int)(_deltaTime / 1000.0);
}

private:

Adafruit_BNO055* _bn055;

float _heading = 0.0;
float _roll = 0.0;
float _pitch = 0.0;
unsigned char _calibrationSys = 0;
unsigned char _calibrationGyro = 0;
unsigned char _calibrationAccel = 0;
unsigned char _calibrationMag = 0;

bool _inMotion = false;

double _accSpeedX = 0.0;
double _accSpeedY = 0.0;
double _accSpeedZ = 0.0;

double _accDistX = 0.0;
double _accDistY = 0.0;
double _accDistZ = 0.0;

double _gyroDegreesX = 0.0;
double _gyroDegreesY = 0.0;
double _gyroDegreesZ = 0.0;

int8_t _temperature = 0;

unsigned long _milliseconds = 0;
unsigned short _microseconds = 0;

unsigned long _prevMilliseconds = 0;
unsigned short _prevMicroseconds = 0;

unsigned long _deltaTime = 0; // Time in microseconds between now an previous.

const double OFFSET_ACCEL_X = 0.01;
const double OFFSET_ACCEL_Y = 00.01;
const double OFFSET_ACCEL_Z = 0.13;
};

// Format sensor data and other platform data into messages to send to the PC:
void sendSystemMessage(InertialController* ctrl) {
    String message = String("i:");
    message = message
            + ctrl->getHeading() + ","
            + ctrl->getPitch() + ","
            + ctrl->getRoll() + ","
            // What speed are we going according to the accelerometer
            + ctrl->getAccSpeedX() + ","
            + ctrl->getAccSpeedY() + ","
            + ctrl->getAccSpeedZ() + ","
            // How far have we gone according to the accelerometer
            + ctrl->getAccDistX() + ","
            + ctrl->getAccDistY() + ","
            + ctrl->getAccDistZ() + ","
            // How much have we rotated.
            + ctrl->getGyroDegreesX() + ","
            + ctrl->getGyroDegreesY() + ","
            + ctrl->getGyroDegreesZ() + ","
            // Are we moving? Consider packing these values into a short int.
            + ctrl->getInMotion() + "," // Byte 1: in motion + system cal + gyro cal
            // Calibration status
            + ctrl->getCalibrationSys() + ","
            + ctrl->getCalibrationGyro() + ","
            + ctrl->getCalibrationAccel() + "," // Byte 2: Accel cal + mag cal
            + ctrl->getCalibrationMag() + ","
            + ctrl->getTemperature() + ","
            + ctrl->getMilliseconds() + ","
            + ctrl->getMicroseconds() + ",\n";

    Serial.print(message);
}

/**
 * This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
 *  which provides a common 'type' for sensor data and some helper functions.
 *
 * To use this driver you will also need to download the Adafruit_Sensor
 * library and include it in your libraries folder.
 *
 * You should also assign a unique ID to this sensor for use with
 * the Adafruit Sensor API so that you can identify this particular
 * sensor in any data logs, etc.  To assign a unique ID, simply
 * provide an appropriate value in the constructor below (12345
 * is used by default in this example).
 *
 * Connections
 * ===========
 * Connect SCL to analog 5
 * Connect SDA to analog 4
 * Connect VDD to 3.3-5V DC
 * Connect GROUND to common ground
 */

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

InertialController* controller;

/**************************************************************************/
/**
 * Arduino setup function (automatically called at startup)
 *
 **************************************************************************/
void setup(void) {
    Serial.begin(57600);
    Serial.print("Orientation Sensor Test\n");
    controller = new InertialController();
    controller->displaySensorDetails();
    pinMode(A2, OUTPUT);
    delay(1000);
}

/**************************************************************************/
/**
 * Arduino loop function, called once 'setup' is complete (your own code
 * should go here)
 *
 ****************************************************************************/
void loop(void) {
    controller->pollSensorData();
    sendSystemMessage(controller);

    controller->setTimestamp(millis(), micros() % 1000);

    int delta = BNO055_SAMPLERATE_DELAY_MS - (int)controller->getDeltaMilliseconds();
    if(delta > 0) {
        delay(BNO055_SAMPLERATE_DELAY_MS - delta);
    }
}
