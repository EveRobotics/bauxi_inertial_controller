#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

unsigned long microseconds = 0;
unsigned long milliseconds = 0;

class InertialController {

public:

InertialController(void) {
    _bn055 = new Adafruit_BNO055(55);
    /* Initialize the sensor */
    if(!_bn055->begin()) {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }
    /* Use external crystal for better accuracy */
    _bn055->setExtCrystalUse(false);
}

/* Display some basic information on this sensor */
void displaySensorDetails(void) {
    sensor_t sensor;
    _bn055->getSensor(&sensor);
    Serial.print("------------------------------------\n");
    Serial.print("Sensor:     ");
    Serial.print(sensor.name);
    Serial.print("\n");
    Serial.print("Driver Ver: ");
    Serial.print(sensor.version);
    Serial.print("\n");
    Serial.print("Unique ID:  ");
    Serial.print(sensor.sensor_id);
    Serial.print("\n");
    Serial.print("Max Value:  ");
    Serial.print(sensor.max_value);
    Serial.print(" xxx\n");
    Serial.print("Min Value:  ");
    Serial.print(sensor.min_value);
    Serial.print(" xxx\n");
    Serial.print("Resolution: ");
    Serial.print(sensor.resolution);
    Serial.print(" xxx\n");
    Serial.print("------------------------------------\n");
    Serial.print("\n");
    delay(500);
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

    if(_calibrationSys != 0) {
        digitalWrite(A2, HIGH);
    } else {
        digitalWrite(A2, LOW);
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

private:

Adafruit_BNO055* _bn055;

float _heading = 0.0;
float _roll = 0.0;
float _pitch = 0.0;
unsigned char _calibrationSys = 0;
unsigned char _calibrationGyro = 0;
unsigned char _calibrationAccel = 0;
unsigned char _calibrationMag = 0;
int8_t _temperature = 0;

unsigned long _milliseconds = 0;
unsigned short _microseconds = 0;

unsigned long _prevMilliseconds = 0;
unsigned short _prevMicroseconds = 0;

unsigned long _deltaTime = 0; // Time in microseconds between now an previous.
};

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void) {

}

// Format sensor data and other platform data into messages to send to the PC:
void sendSystemMessage(InertialController* ctrl) {
    String message = String("i:");
    message = message
            + ctrl->getHeading() + ","
            + ctrl->getPitch() + ","
            + ctrl->getRoll() + ","
            + ctrl->getCalibrationSys() + ","
            + ctrl->getCalibrationGyro() + ","
            + ctrl->getCalibrationAccel() + ","
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
 *
 * History
 * =======
 * 2015/MAR/03  - First release (KTOWN)
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
    Serial.println("Orientation Sensor Test"); Serial.println("");
    controller = new InertialController();
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
    delay(BNO055_SAMPLERATE_DELAY_MS);
    controller->setTimestamp(millis(), micros() % 1000);
}
