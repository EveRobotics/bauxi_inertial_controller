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
}

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
    delay(10);
}

