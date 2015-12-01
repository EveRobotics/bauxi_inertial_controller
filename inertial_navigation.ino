
/** @file inertial_navigation.ino
 *  @brief Beagle Inertial Navigation controller.
 *
 *  @author Weston Turner
 */

#include <Wire.h>
#include "Accel_ADXL345.h"  // ADXL345 Accelerometer Library
#include "Mag_HMC5883L.h" // HMC5883L Magnetometer Library
#include "Gyro_ITG3200.h"  // Rate gyro
#include "moving_average.h"

#include <math.h>
#include <EEPROM.h>

void(* resetFunc) (void) = 0; //declare reset function @ address 0

// Process received serial data for configuration and control commands:
class CommandInterpreter {

public:
    CommandInterpreter() {
        // Process instructions from the system controller:
        // set/get mag offset X
        // set/get mag offset Y
        // set/get mag offset Z
        // etc..
    }

    void readCommands() {
        while(Serial.available() > 0) {
            // read the incoming bytes:
            unsigned char command = Serial.read();
            Serial.print("Received command: ");
            Serial.print(command);
            Serial.print(", param: ");
            if(Serial.read() == ':') {
                if(command == CMD_MAG_OFFSET_X) {
                    _magOffsetX = Serial.parseFloat();
                    Serial.println(_magOffsetX);
                } else if(command == CMD_RESET) {
                    Serial.println("Resetting inertial navigation controller.");    
                    resetFunc();  //call reset
                }
                // Parse user indicator commands and configuration commands.
                if(Serial.read() != ';') {
                    Serial.println("Received invalid end of message.");    
                }
            }
        }
    }

    int getMagX() {
        return _magOffsetX;
    }

    int getMagY() {
        return _magOffsetY;
    }
    
    int getMagZ() {
        return _magOffsetZ;
    }


private:

    enum InputCommands {
        CMD_RESET           = 0x61, // 'a' abort, reset the micro-controller.
        CMD_MAG_OFFSET_X    = 0x6d, // 'm' magnetic field sensor X offset.
        CMD_MAG_OFFSET_Y    = 0x6e, // 'n' magnetic field sensor Y offset.
        CMD_MAG_OFFSET_Z    = 0x6f, // 'o' magnetic field sensor Z offset.
        // ...
    };

    float _magOffsetX = 0.0;
    float _magOffsetY = 0.0;
    float _magOffsetZ = 0.0;

};


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Circle Center Finder Class and helper classes.
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

class Point {
    
public:
    
    Point(void) { }
    
    Point(double x, double y, double z) {
        _x = x;
        _y = y;
        _z = z;
    }
    
    void setX(double x) {
        _x = x;
    }
        
    void setY(double y) {
        _y = y;
    }
    
    void setZ(double z) {
        _z = z;
    }
        
    double getX(void) {
        return _x;
    }
        
    double getY(void) {
        return _y;
    }
    
    double getZ(void) {
        return _z;
    }
  
private:
    double _x = 0.0;
    double _y = 0.0;
    double _z = 0.0;  
};


class Circle {

public:
    
    Circle(void) { return; }
    
    // p1, p2, p3 are co-planar  
    Circle(Point p1, Point p2, Point p3) {
        _radius = -1;

        if(!_isPerpendicular(p1, p2, p3)) {
            _calcCircle(p1, p2, p3);    
        } else if(!_isPerpendicular(p1, p3, p2)) {
            _calcCircle(p1, p3, p2);   
        } else if(!_isPerpendicular(p2, p1, p3)) {
            _calcCircle(p2, p1, p3);
        } else if(! _isPerpendicular(p2, p3, p1)) {
            _calcCircle(p2, p3, p1);    
        } else if(!_isPerpendicular(p3, p2, p1)) {
            _calcCircle(p3, p2, p1);
        } else if(!_isPerpendicular(p3, p1, p2)) {
            _calcCircle(p3, p1, p2);
        } else {
            _radius = -1;
        }
        return;
    }
    
    virtual ~Circle() { }

    double getRadius(void) {
        return _radius;
    }
    
    Point getCenter(void) {
        return _center;
    }
    
private:

    double _radius;
    Point _center;

    double _calcCircle(Point p1, Point p2, Point p3) {
        double yDeltaA = p2.getY() - p1.getY();
        double xDeltaA = p2.getX() - p1.getX();
        double yDeltaB = p3.getY() - p2.getY();
        double xDeltaB = p3.getX() - p2.getX();
    
        if(abs(xDeltaA) <= 0.000000001 && abs(yDeltaB) <= 0.000000001) {
            _center.setX(0.5 * (p2.getX() + p3.getX()));
            _center.setY(0.5 * (p1.getY() + p2.getY()));
            _center.setZ(p1.getZ());
            // Calculate the circle's radius.
            _radius = _distance(_center, p1);
            return _radius;
        }
        
        // IsPerpendicular() assure that xDelta(s) are not zero
        double aSlope = yDeltaA / xDeltaA;
        double bSlope = yDeltaB / xDeltaB;
        // Checking whether the given points are colinear. 
        if(abs(aSlope - bSlope) <= 0.000000001) {    
            return -1;
        }

        // Calculate circle center:
        _center.setX((aSlope * bSlope * (p1.getY() - p3.getY()) + bSlope
            * (p1.getX() + p2.getX()) - aSlope * (p2.getX() + p3.getX()))
            / (2.0 * (bSlope - aSlope)));
        
        _center.setY(-1.0 * (_center.getX() - (p1.getX() + p2.getX())
            / 2.0) / aSlope + (p1.getY() + p2.getY() / 2.0));
        
        _center.setZ(p1.getZ());
        // Calculate the circle's radius.
        _radius = _distance(_center, p1);
        
        return _radius;
    }
    
    bool _isPerpendicular(Point p1, Point p2, Point p3) {
        // Check the given point are perpendicular to x or y axis.
        double yDeltaA = p2.getY() - p1.getY();
        double xDeltaA = p2.getX() - p1.getX();
        double yDeltaB = p3.getY() - p2.getY();
        double xDeltaB = p3.getX() - p2.getX();

        // Checking whether the line of the two points are vertical.
        if(abs(xDeltaA) <= 0.000000001 && abs(yDeltaB) <= 0.000000001) {
            return false;
        }
        
        if(abs(yDeltaA) <= 0.0000001) {
            return true;
        } else if(abs(yDeltaA) <= 0.0000001) {
            return true;
        } else if(abs(xDeltaA) <= 0.000000001) {
            return true;
        } else if(abs(xDeltaA) <= 0.000000001) {
            return true;
        } else {
            return false;
        }
        return false;
    }
    
    
    double _distance(Point p1, Point p2) {
        // Compute the distance between the two points
        return sqrt((p1.getX() - p2.getX()) * (p1.getX() - p2.getX())
            + (p1.getY() - p2.getY()) * (p1.getY() - p2.getY()) 
            + (p1.getZ() - p2.getZ()) * (p1.getZ() - p2.getZ()));
    }
};
    



class CircleCenterFinder {
    // http://paulbourke.net/geometry/circlesphere/
    // Here we implement the circle center location logic.
    
public:
    CircleCenterFinder() {
        // 
        for(int i = 0; i < LIST_SIZE; i++) {
            _angleList[i] = -1; // Sentinel value.
        }
    }

    bool addPoint(double x, double y, double z, double angle) {
        int iAngle = 0;
        if(angle < 0) {
            iAngle = (int)((long)angle % 360) + 360;
        } else {
            iAngle = (int)((long)angle % 360);
        }
        iAngle = iAngle / 36;
        _pointList[iAngle].setX(x);
        _pointList[iAngle].setY(y);
        _pointList[iAngle].setZ(z);
        _angleList[iAngle] = iAngle;
        int index1 = -1;
        int index2 = -1;
        // Now try to get 2 points adjacent to iAngle from the point list.

        if(_angleList[(iAngle - 1) % 10] != -1) {
            // -30 degrees:
            index1 = (iAngle - 1) % 10;
            // Set index 2 to index 1 in case we cant initialize in > 0 degrees.
            index2 = index1;
        }
        // Set index 1 to -60 degrees if possible.
        if(_angleList[(iAngle - 2) % 10] != -1) {
            // -60 degrees:
            index1 = (iAngle - 2) % 10;
        }
        // Maximize index 1 up to -90 degrees.
        if(_angleList[(iAngle - 3) % 10] != -1) {
            // -90 degrees:
            index1 = (iAngle - 3) % 10;
        } else if(index1 == index2) {
            // If we couldn't set index 1 != index 2 then set index 2 to -1.
            index2 = -1;
        }
        // > 0 degrees:
        // Set index 2. Start at 30 degrees from zero (zero is the new index).
        if(_angleList[(iAngle + 1) % 10] != -1) {
            // +30 degrees
            index2 = (iAngle + 1) % 10;
            if(index1 == -1) {
                // If we haven't initialized index 1 above, initialize it here.
                index1 = index2;
            }
        }
        // Increase the angle to 60 degreees if possible. 60 is better than 30.
        if(_angleList[(iAngle + 2) % 10] != -1) {
            // +60 degrees.
            index2 = (iAngle + 2) % 10;
        }
        // Attempt to maximize the angle up to 90 degrees.
        if(_angleList[(iAngle + 3) % 10] != -1) {
            // +90 degrees.
            index2 = (iAngle + 3) % 10;
        } else if(index1 == index2) {
            // If index 1 equals index 2 then we cannot compute the circle 
            // center because we don't have 3 unique points. 
            index1 = -1;
            index2 = -1;
        }

        bool circleCalculated = false;
        if(index1 != -1 && index2 != -1) {
            _circle = Circle(_pointList[index1]
                , _pointList[iAngle], _pointList[index2]);
            
            circleCalculated = true;
        }
        
        return circleCalculated;
    }

    Circle getCircle(void) {
        return _circle;
    }

private:
    const int LIST_SIZE = 12;
    Point _pointList[12];
    int _angleList[12];
    Circle _circle;
    bool _trained = false;
};



int testFloat(float f) {
    if((f == NAN) || (f == INFINITY) || (-f == INFINITY) 
        || (isnan(f)) || (isinf(f))) {
        return -1;
    } else if(isfinite(f)) {  
        return 0;
    } else {
        return -1;
    }
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Arduino setup and loop functions:
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

HMC5883L compass;
// Variable adxl is an instance of the ADXL345 library.
ADXL345 acc;
ITG3200 gyro = ITG3200();

// Compass variables (magnetometer):
float magX = 0, magY = 0, magZ = 0;
// Compass variables (magnetometer) adjusted:
double adjMagX = 0, adjMagY = 0, adjMagZ = 0;
// Exponential weighted moving average.
double expMagX = 0, expMagY = 0, expMagZ = 0;

float magOffsetX = 0.0; //
float magOffsetY = 0.0; //
float magOffsetZ = 0.0; //

// Accelerometer variables:
int accX, accY, accZ;
// Accelerometer variables:
double adjAccX, adjAccY, adjAccZ;

float accOffsetX = 0.0;
float accOffsetY = 0.0;
float accOffsetZ = 0.0;

// First integration is speed.
double accSpeedX = 0.0;
double accSpeedY = 0.0;
double accSpeedZ = 0.0;

// Second integration is distance.
double accDistX = 0.0;
double accDistY = 0.0;
double accDistZ = 0.0;

float accPitch = 0.0;
float accRoll = 0.0;

// Rate gyroscope variables:
float gyroX, gyroY, gyroZ;
// Rate gyroscope variables:
double adjGyroX, adjGyroY, adjGyroZ;

float gyroOffsetX = 0.0;
float gyroOffsetY = 0.0;
float gyroOffsetZ = 0.0;

double gyroDegreesX = 0.0;
double gyroDegreesY = 0.0;
double gyroDegreesZ = 0.0;

double prevGyroDegreesZ = 0.0;


unsigned long milliseconds = 0;
unsigned int microseconds = 0;
double time0 = 0.0;
double time1 = 0.0;


float *offsetTable[] = { &magOffsetX, &magOffsetY, &magOffsetZ
    , &accOffsetX, &accOffsetY, &accOffsetY
    , &gyroOffsetX, &gyroOffsetY, &gyroOffsetZ };

// The "i" message is the inertial navigation message.
String iMessage;

MovingAverage *magWmaX;
MovingAverage *magWmaY;
//MovingAverage *magWmaZ;

MovingAverage *accWmaX;
MovingAverage *accWmaY;
MovingAverage *accWmaZ;

MovingAverage *gyroWmaX;
MovingAverage *gyroWmaY;
MovingAverage *gyroWmaZ;

MovingAverage *magCenterWmaX;
MovingAverage *magCenterWmaY;

int error = 0; 
unsigned long time, looptime;

int eeAddress = 0;   //Location we want the data to be put.

void setup() {
    Serial.begin(57600);

    milliseconds = millis();
    microseconds = micros() % 1000;
    time0 = (double)milliseconds + ((double)microseconds / 1000.0);

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
    magWmaX = new ExponentialMovingAverage();
    magWmaY = new ExponentialMovingAverage();
    //magWmaZ = new ExponentialMovingAverage();

    accWmaX = new ExponentialMovingAverage();
    accWmaY = new ExponentialMovingAverage();
    accWmaZ = new ExponentialMovingAverage();

    gyroWmaX = new ExponentialMovingAverage();
    gyroWmaY = new ExponentialMovingAverage();
    gyroWmaZ = new ExponentialMovingAverage();
    
    // Slow down the movement of the average with a small alpha.
    magCenterWmaX = new ExponentialMovingAverage(0.05);
    magCenterWmaY = new ExponentialMovingAverage(0.05);
    
    eeAddress = 0;
    for(; &gyroOffsetZ != offsetTable[eeAddress]; eeAddress += sizeof(float)) {
        EEPROM.get(eeAddress, *offsetTable[eeAddress]);
        if(testFloat(*offsetTable[eeAddress]) != 0) {
            *offsetTable[eeAddress] = 0.0;
        }
    }

    // We want to print the retrieved stored value, but we are out of program space.
    
    if(magOffsetX != 0.0) {
        magCenterWmaX->addSample(-magOffsetX);
        //Serial.print("Initialized mag offset (X, Y) to: ");
        //Serial.print(magOffsetX);
    }

    if(magOffsetY != 0.0) {
        magCenterWmaY->addSample(-magOffsetY);
        //Serial.println("Initialized mag offset Y to:");
        //Serial.println(magOffsetY);
    }

    delay(60);
}


float heading = 0.0;

// Time in microseconds between now an previous.
double deltaTime = 0.0;

// 255 / 9.8 m/s^2
double accRatio = 26.020408163265305;

const float MAG_MAX_VAL = 1500.0;
const float MAG_CONT_THRESH = 100.0;
const float MAG_DELTA_THRESH = 7.0;
const float ACC_DELTA_THRESH = 2.5;
const float GYRO_DELTA_THRESH = 3.2;

const float ACC_THRESH_1 = 10.8; // Uncalibrated, threshold value to indicate movement.
const float ACC_THRESH_2 = 9.85; // Calibrated threshold value to indicate movement.
const float GYRO_THRESH_1 = 6.0; // Uncalibrated magnitude beyond which motion is occuring.
const float GYRO_THRESH_2 = 1.8; // Calibrated magnitude motion threshold. 

double magDist = 0.0, accDist = 0.0, gyroDist = 0.0;
double prevMagDist = 0.0, prevAccDist = 0.0, prevGyroDist = 0.0;
bool inMotion = true;
int atRestCount = 0;

bool calibratedAccGyro = false;
bool calibratedCompass = false;

// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
//const float DECLINATION = -8.58; // Declination (degrees) in Boulder, CO.
const float DECLINATION = -15.2833333333333333;

CircleCenterFinder centerFinder;


void loop() {
    // Store the time-stamp.
    milliseconds = millis();
    microseconds = micros() % 1000;
    time1 = (double)milliseconds + ((double)microseconds / 1000.0);
    deltaTime = (time1 - time0)  / 1000.0; // Convert to seconds.
    time0 = time1;

    // Code fragment for Magnetometer heading (yaw)
    MagnetometerRaw raw = compass.ReadRawAxis();
    // TODO: Figure out what scaled does.
    MagnetometerScaled scaled = compass.ReadScaledAxis();
    // Compass variable init:
    magX = scaled.XAxis; // Raw megnetometer X value.
    magY = scaled.YAxis;
    magZ = scaled.ZAxis;

    // Read the accelerometer values and store them in variables  x, y, z:
    acc.readAccel(&accX, &accY, &accZ);
  
    // Code fragment for Gyroscope (roll, pitch, yaw)  
    gyro.readGyro(&gyroX, &gyroY, &gyroZ);
    gyroZ = gyroZ * -1.0; // Sensor is flipped upside-down.
    
    // Accelerometer, Magnetometer and Gyroscope Output
    iMessage = "i:";

    // Add the megnetometer data to the message.
    iMessage = iMessage + (float)magX + ",";
    iMessage = iMessage + (float)magY + ",";
    iMessage = iMessage + (float)magZ + ",";
    // Add the acceleration data to the message.
    iMessage = iMessage + (int)accX + ",";
    iMessage = iMessage + (int)accY + ",";
    iMessage = iMessage + (int)accZ + ",";
    // Gyro data.
    iMessage = iMessage + (float)gyroX + ",";
    iMessage = iMessage + (float)gyroY + ",";
    iMessage = iMessage + (float)gyroZ + ",";


    // Add the time-stamp to the string:
    iMessage = iMessage + (unsigned long)milliseconds + ",";
    //iMessage = iMessage + (int)microseconds + ",";
    
    //==========================================================================
    // MAGNETOMETER CALCULATIONS:
    //==========================================================================
    // Since we are not integrating this, we can filter it...
    // Do not update values if they are not within 10 percent of average.
    // But what if we started out with wacky data. then nothing will be accepted...
    // 0.01 is just an epsilon here (we dont want to divide by zero.
    
    // Filter noise spikes, transient peaks.
    if ((abs(magX) < MAG_MAX_VAL && abs(magY) < MAG_MAX_VAL 
        && abs(magZ) < MAG_MAX_VAL)) {
        
        magWmaX->addSample(magX);
        magWmaY->addSample(magY);
        //magWmaZ->addSample(magZ);
        // Filter discontinuous motion (impossible rates of change).
        if (sqrt((expMagX - magX) * (expMagX - magX) 
            + (expMagY - magY) * (expMagY - magY)) 
            //+ (expMagZ - magZ) * (expMagZ - magZ)) 
            < MAG_CONT_THRESH) {
            
            adjMagX = expMagX + magOffsetX;
            adjMagY = expMagY + magOffsetY;
            adjMagZ = expMagZ + magOffsetZ;
        }
        
        expMagX = magWmaX->computeAverage();
        expMagY = magWmaY->computeAverage();
        //expMagZ = magWmaZ->computeAverage();
    }

    // Take the raw data and convert it to magnetometer yaw (heading)
    // https://en.wikipedia.org/wiki/Atan2
    if (adjMagY == 0) {
        heading = (adjMagX < 0) ? PI : 0;
    } else {
        heading = atan2(adjMagY, adjMagX);
    }
    // Declination is in degrees, heading is in radians.
    heading += DECLINATION * (PI / 180.0);

    if (heading > (2.0 * PI)) {
        heading -= (2.0 * PI);
    } else if (heading < 0) {
        heading += (2.0 * PI);
    } 

    // Convert to degrees.
    heading *= (180.0 / PI);

    //==========================================================================
    // ACCELEROMETER CALCULATIONS:
    //==========================================================================
    // Compute acceleration in meters per second squared.
    // 255 = 9.8 m/s^2
    adjAccX = (double)(accX) / accRatio;
    adjAccY = (double)(accY) / accRatio;
    adjAccZ = (double)(accZ) / accRatio;

    accWmaX->addSample(adjAccX);
    accWmaY->addSample(adjAccY);
    accWmaZ->addSample(adjAccZ);
    
    // Also calculate pitch and roll using:
    if (adjAccX == 0.0) {
        accPitch = (adjAccZ < 0.0) ? PI : 0.0;
    } else {
        accPitch = atan2(adjAccX, adjAccZ);
    }
    
    if (adjAccY == 0.0) {
        accRoll = (adjAccZ < 0.0) ? PI : 0.0;
    } else {
        accRoll = atan2(adjAccY, adjAccZ);
    }

    if (accPitch > (2.0 * PI)) {
        accPitch -= (2.0 * PI);
    } else if (accPitch < 0.0) {
        accPitch += (2.0 * PI);
    }
    
    if (accRoll > (2.0 * PI)) {
        accRoll -= (2.0 * PI);
    } else if (accRoll < 0.0) {
        accRoll += (2.0 * PI);
    } 
    
    // Convert to degrees.
    accPitch *= (180.0 / PI);
    // Convert to degrees.
    accRoll *= (180.0 / PI);

    // Integrate accelerometer data once to get speed and twice for distance.
    if(calibratedAccGyro) {
        if(accDist > ACC_THRESH_2) {
            accSpeedX = accSpeedX + ((adjAccX + accOffsetX) * deltaTime);
            accSpeedY = accSpeedY + ((adjAccY + accOffsetY) * deltaTime);
            accSpeedZ = accSpeedZ + ((adjAccZ + accOffsetZ) * deltaTime);
            // Second integral:
            accDistX = accDistX + (accSpeedX * deltaTime);
            accDistY = accDistY + (accSpeedY * deltaTime);
            accDistZ = accDistZ + (accSpeedZ * deltaTime);
        } else {
            // If we are at rest, then reduce the speeds towards zero.
            accSpeedX = accSpeedX * 0.99;
            accSpeedY = accSpeedY * 0.99;
            accSpeedZ = accSpeedZ * 0.99;
        }
    }

    //==========================================================================
    // GYRO CALCULATIONS:
    //==========================================================================
    gyroWmaX->addSample(gyroX);
    gyroWmaY->addSample(gyroY);
    gyroWmaZ->addSample(gyroZ);
    adjGyroX = (double)(gyroX + gyroOffsetX);
    adjGyroY = (double)(gyroY + gyroOffsetY);
    adjGyroZ = (double)(gyroZ + gyroOffsetZ);
    	
    // Compute pose using rate gyro, integrate the degrees per second 
    // to absolute degrees (X, Y, Z).
    // Multiply the degrees per second by the elapsed time.
    if(calibratedAccGyro) {
        gyroDegreesX = gyroDegreesX + (adjGyroX * deltaTime);
        gyroDegreesY = gyroDegreesY + (adjGyroY * deltaTime);
        gyroDegreesZ = gyroDegreesZ + (adjGyroZ * deltaTime);
    }

    //==========================================================================
    // CALCULATE SENSOR DISTANCES:
    //==========================================================================
    magDist = sqrt(adjMagX * adjMagX + adjMagY * adjMagY + adjMagZ * adjMagZ);
    accDist = sqrt(adjAccX * adjAccX + adjAccY * adjAccY + adjAccZ * adjAccZ);
    gyroDist = sqrt(adjGyroX * adjGyroX + adjGyroY * adjGyroY + adjGyroZ * adjGyroZ);

    if(abs(magDist - prevMagDist) > MAG_DELTA_THRESH
        || abs(accDist - prevAccDist) > ACC_DELTA_THRESH
        || abs(gyroDist - prevGyroDist) > GYRO_DELTA_THRESH
        || accDist > ACC_THRESH_1 || gyroDist > GYRO_THRESH_1
        // Apply the smaller threshold if the offsets have been calulated.
        // We could just look at the Z axis acceleration for this...
        || (gyroDist > GYRO_THRESH_2 && (gyroOffsetX != 0.0 || gyroOffsetY != 0.0))) {
        
        if(abs(gyroDegreesZ - prevGyroDegreesZ) > (GYRO_DELTA_THRESH * 5.0)) {
            // 
            prevGyroDegreesZ = gyroDegreesZ;
            if(centerFinder.addPoint(expMagX, expMagY, expMagZ, gyroDegreesZ)) {
                double mX = centerFinder.getCircle().getCenter().getX();
                double mY = centerFinder.getCircle().getCenter().getY();
                double mZ = centerFinder.getCircle().getCenter().getZ();
                // Reject noisy measurements.
                if(abs(mX) < (MAG_MAX_VAL / 2) && abs(mY) < (MAG_MAX_VAL / 2) 
                    && abs(mZ) < (MAG_MAX_VAL / 2)) {
                    magCenterWmaX->addSample(mX);
                    magCenterWmaY->addSample(mY);
                    magOffsetX = -magCenterWmaX->computeAverage();
                    magOffsetY = -magCenterWmaY->computeAverage();
                    calibratedCompass = true;
                }
            }
        }

        atRestCount = 0;
        inMotion = true;
        
    } else {
        if(atRestCount > 10) {
            inMotion = false;
            accOffsetX = -accWmaX->computeAverage();
            accOffsetY = -accWmaY->computeAverage();
            accOffsetZ = -accWmaZ->computeAverage();
            gyroOffsetX = -gyroWmaX->computeAverage();
            gyroOffsetY = -gyroWmaY->computeAverage();
            gyroOffsetZ = -gyroWmaZ->computeAverage();
            calibratedAccGyro = true;
        } else if(atRestCount == 10) {
            // We store these values in the EEPROM in in adition to the
            // magOffsetX, magOffsetY, and magOffsetZ values.
            eeAddress = 0;
            for(; &gyroOffsetZ != offsetTable[eeAddress]; eeAddress += sizeof(float)) {
                EEPROM.put(eeAddress, *offsetTable[eeAddress]);
            }
            //Serial.println("Storing mag offset X as:");
            //Serial.println(magOffsetX);
            //Serial.println("Storing mag offset Y as:");
            //Serial.println(magOffsetY);
            atRestCount++;
        } else {
            atRestCount++;
        }
    }

    digitalWrite(10, inMotion);
    
    prevMagDist = magDist;
    prevAccDist = accDist;
    prevGyroDist = gyroDist;
    

    //==========================================================================
    // PRINT CALCULATED VALUES:
    //==========================================================================
    iMessage = iMessage + (double)adjMagX + ",";
    iMessage = iMessage + (double)adjMagY + ",";
    iMessage = iMessage + (float)heading + ",";
    
    iMessage = iMessage + (float)accPitch + ",";
    iMessage = iMessage + (float)accRoll + ",";

    iMessage = iMessage + (double)accSpeedX + ",";
    iMessage = iMessage + (double)accSpeedY + ",";
    iMessage = iMessage + (double)accSpeedZ + ",";
    
    iMessage = iMessage + (double)accDistX + ",";
    iMessage = iMessage + (double)accDistY + ",";
    iMessage = iMessage + (double)accDistZ + ",";
    
    iMessage = iMessage + (double)gyroDegreesX + ",";
    iMessage = iMessage + (double)gyroDegreesY + ",";
    iMessage = iMessage + (double)gyroDegreesZ + ",";
    
    iMessage = iMessage + (double)deltaTime + ",";
    iMessage = iMessage + (unsigned int)(inMotion 
        + (calibratedAccGyro << 1)
        + (calibratedCompass << 2)) + ",";
    Serial.println(iMessage);
    
    delay(80);
    //delay(100 - (int)(deltaTime * 1000.0));
}

