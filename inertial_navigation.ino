// Arduino code and libraries are available to download - link below the video.

// 

/*
Arduino     MARG GY-85
    A5            SCL
    A4            SDA
    3.3V          VCC
    GND           GND
*/ 

#include <Wire.h>
#include "Accel_ADXL345.h"  // ADXL345 Accelerometer Library
#include "Mag_HMC5883L.h" // HMC5883L Magnetometer Library
#include "Gyro_ITG3200.h"  // Rate gyro
#include "moving_average.h"


#include <math.h>


// Process received serial data for configuration and control commands:
class CommandInterpreter {

public:
    CommandInterpreter() {
        // Process instructions from the system controller:
        // set mag offset X
        // set mag offset Y
        // set mag offset Z

        // set mag offset Z


    }

    void readCommands() {
        while(Serial.available() > 0) {
            // read the incoming bytes:
            unsigned char command = Serial.read();
            Serial.print("Received command: ");
            Serial.print(command);
            Serial.print(", param: ");
            if(Serial.read() == ':') {
                if(command == CMD_SPEED_LEFT) {
                    _speedLeft = Serial.parseInt();
                    Serial.println(_speedLeft);
                } else if(command == CMD_SPEED_RIGHT) {
                    _speedRight = Serial.parseInt();
                    Serial.println(_speedRight);
                } else if(command == CMD_RUN_MODE) {
                    _runMode = Serial.parseInt();
                    Serial.println(_runMode);
                } else if(command == CMD_RESET) {
                    Serial.println("Resetting spacial navigation controller.");    
                    resetFunc();  //call reset
                }
                // Parse user indicator commands and configuration commands.
                if(Serial.read() != ';') {
                    Serial.println("Received invalid end of message.");    
                }
            }
        }
    }

    int getSpeedLeft() {
        return _speedLeft;
    }

    int getSpeedRight() {
        return _speedRight;
    }
    
    int getRunMode() {
        return _runMode;
    }


private:

    enum InputCommands {
        CMD_RESET           = 0x61, // 'a' abort, reset the micro-controller.
        CMD_SPEED_LEFT      = 0x6c, // 'l' left, valid speeds: (-10, 10)
        CMD_SPEED_RIGHT     = 0x72, // 'r' right, valid speeds: (-10, 10)
        CMD_RUN_MODE        = 0x6d, // 'm' mode: 0 = AUTO_AVOID, 1 = EXT_CONTROL, 2 = REMOTE_CONTROL
        // ...
    };

    int _speedLeft;
    int _speedRight;
    int _runMode;

    unsigned char _userIndicator1;
    unsigned char _userIndicator2;
    unsigned char _userIndicator3;
    unsigned char _userIndicator4;

};



HMC5883L compass;
// Variable adxl is an instance of the ADXL345 library.
ADXL345 acc;
ITG3200 gyro = ITG3200();


// Compass variables (magnetometer):
float magX, magY, magZ;

// Accelerometer variables:
int accX, accY, accZ;

// Rate gyroscope variables:
float gyroX, gyroY, gyroZ;

unsigned long milliseconds;
unsigned int microseconds;
unsigned long prevMilliseconds;
unsigned int prevMicroseconds;
// Time in microseconds between now an previous.
unsigned long deltaTime = 0; 
// The "i" message is the inertial navigation message.
String iMessage;

WeightedMovingAverage *magWmaX;
WeightedMovingAverage *magWmaY;
WeightedMovingAverage *magWmaZ;

WeightedMovingAverage *accWmaX;
WeightedMovingAverage *accWmaY;
WeightedMovingAverage *accWmaZ;

WeightedMovingAverage *gyroWmaX;
WeightedMovingAverage *gyroWmaY;
WeightedMovingAverage *gyroWmaZ;

int error = 0; 
unsigned long time, looptime;

void setup() {
    Serial.begin(115200);

    // These are the interrupt inputs, we may not use them.
    pinMode(4, INPUT);
    pinMode(5, INPUT);
    pinMode(6, INPUT);

    iMessage = String("i");

    // Power on the accelerometer? Is this just initialization?
    acc.powerOn();
    // Constructor.
    compass = HMC5883L();

    // Set the scale to +/- 1.3 Ga of the compass.
    error = compass.SetScale(1.3);
    if(error != 0) { // If there is an error, print it out.
        Serial.println(compass.GetErrorText(error));
    }

    // Set the measurement mode to Continuous
    error = compass.SetMeasurementMode(Measurement_Continuous);
    if(error != 0) { 
        // If there is an error, print it out.
        Serial.println(compass.GetErrorText(error));
    }

    gyro.init(ITG3200_ADDR_AD0_LOW);

    // Set a status LED to indicate we started up, indicate
    // if there was an initialization error.


    magWmaX = new WeightedMovingAverage(20);
    magWmaY = new WeightedMovingAverage(20);
    magWmaZ = new WeightedMovingAverage(20);

    /*
    accWmaX = new WeightedMovingAverage(5);
    accWmaY = new WeightedMovingAverage(5);
    accWmaZ = new WeightedMovingAverage(5);

    gyroWmaX = new WeightedMovingAverage(5);
    gyroWmaY = new WeightedMovingAverage(5);
    gyroWmaZ = new WeightedMovingAverage(5);
    */
}



float heading = 0.0;
float headingRadians = 0.0;
float headingDegrees = 0.0;

// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.

void loop() {
    // Store the time-stamp.
    milliseconds = millis();
    microseconds = micros() % 1000;
    deltaTime = ((milliseconds * 1000) + microseconds) 
        - ((prevMilliseconds * 1000) + prevMicroseconds);

    // Code fragment for Magnetometer heading (yaw)
    MagnetometerRaw raw = compass.ReadRawAxis();
    // TODO: Figure out what scaled does.
    MagnetometerScaled scaled = compass.ReadScaledAxis();
    // Compass variable init:
    magX = scaled.XAxis;
    magY = scaled.YAxis;
    magZ = scaled.ZAxis;

    // Read the accelerometer values and store them in variables  x, y, z:
    acc.readAccel(&accX, &accY, &accZ);
  
    // Code fragment for Gyroscope (roll, pitch, yaw)  
    gyro.readGyro(&gyroX, &gyroY, &gyroZ);
    
    // Determine whether to integrate gyro data 
    //anglegx = anglegx + (gx_rate * looptime);
    //anglegy = anglegy + (gy_rate * looptime);
    //anglegz = anglegz + (gz_rate * looptime);
    // We could also distance by integrating the accel data.
    
    // Accelerometer, Magnetometer and Gyroscope Output
    iMessage = "i:";
    iMessage = iMessage + (float)magX + ",";
    iMessage = iMessage + (float)magY + ",";
    iMessage = iMessage + (float)magZ + ",";
    // Add the acceleration data to the
    iMessage = iMessage + (int)accX + ",";
    iMessage = iMessage + (int)accY + ",";
    iMessage = iMessage + (int)accZ + ",";
    // Gyro data.
    iMessage = iMessage + (float)gyroX + ",";
    iMessage = iMessage + (float)gyroY + ",";
    iMessage = iMessage + (float)gyroZ + ",";
    // Add the time-stamp to the string:
    iMessage = iMessage + (unsigned long)milliseconds + ",";
    iMessage = iMessage + (unsigned int)microseconds;

    Serial.println(iMessage);


    magWmaX->addSample(magX);
    magWmaY->addSample(magY);
    magWmaZ->addSample(magZ);

    iMessage = "a:";
    iMessage = iMessage + magWmaX->computeAverage() + ",";
    iMessage = iMessage + magWmaY->computeAverage() + ",";
    iMessage = iMessage + magWmaZ->computeAverage() + ",";

    /*
    // Add the acceleration data to the
    accWmaX->addSample(accX);
    accWmaY->addSample(accY);
    accWmaZ->addSample(accZ);
    iMessage = iMessage + accWmaX->computeAverage() + ",";
    iMessage = iMessage + accWmaY->computeAverage() + ",";
    iMessage = iMessage + accWmaZ->computeAverage() + ",";
    // Gyro data.
    gyroWmaX->addSample(gyroX);
    gyroWmaY->addSample(gyroY);
    gyroWmaZ->addSample(gyroZ);
    iMessage = iMessage + gyroWmaX->computeAverage() + ",";
    iMessage = iMessage + gyroWmaY->computeAverage() + ",";
    iMessage = iMessage + gyroWmaZ->computeAverage() + ",";
    // Add the time-stamp to the string:
    iMessage = iMessage + (unsigned long)milliseconds + ",";
    iMessage = iMessage + (unsigned int)microseconds;
    */
    Serial.println(iMessage);


    
    /*
    // Compute pose using rate gyro, integrate the degrees per second 
    // to absolute degrees (X, Y, Z).
    
    // Take the raw data and convert it to magnetometer yaw (heading)
    // 
	float heading;
	if (magY == 0)
		heading = (magX < 0) ? 180.0 : 0;
	else
	heading = atan2(magX, magY);

	heading -= DECLINATION * PI / 180;

	if (heading > PI) heading -= (2 * PI);
	else if (heading < -PI) heading += (2 * PI);
	else if (heading < 0) heading += 2 * PI;

	heading *= 180.0 / PI;
  
    Serial.println(heading);
    */
    
    delay(100);
}

