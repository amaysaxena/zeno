/*
  # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
  #                                                                     #
  #     Zeno.cpp - Library for the Zeno educational             #
  #     robotics platform.                                              #
  #     Read included example files in the /examples directory          #
  #     for sample usage.                                               #
  #                                                                     #
  #     Created by Amay S. Saxena, March 31, 2017.                      #
  #     Released into the public domain.                                #
  #                                                                     #
  #     FAQ:                                                            #
  #                                                                     #
  # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
*/

#ifndef Zeno_h
#define Zeno_h

#include "Arduino.h"
#include "Zeno.h"
#include <Wire.h>
#include "Adafruit_MotorShield.h"
//#include "Adafruit_MotorShield/utility/Adafruit_MS_PWMServoDriver.h"
#include <math.h>
#include "Adafruit_9DOF.h"
#include "Adafruit_L3GD20_U.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>

class Zeno {

  public:

    Zeno(bool MotorDPresent);
    /*  Initializes the basic Zeno drivebase, setting the 
     *  drive motor and sensor pins to the appropriate values.
     *  @PARAM MotorDPresent : boolean- set this to true if you have
     *      included a fourth motor in your project 
     *      (apart from the 3 drivebase motors).
     */

    void ZenoSetup();
    /*  Does all the neccessary initialization bits to get the serial
     *  communication going. Put this in the setup() of your Arduino sketch.
     */    

    void moveForward(int speed);
    /* The robot begins mooving forward.
     *  
     *  @PARAM speed : Integer between -255 and 255 inclusive
     *    (Negative speeds represent the reverse direction
     *    i.e. moving backwards)
     */

    void stopMoving();
     /* Stops all robot drivebase motion.
      */

    void motorA(int speed);
    /*  Motor A starts to move at the specified speed.
     *  
     *  @PARAM speed : Integer between -255 and 255 inclusive
     *    (Negative speeds represent the reverse direction
     *    i.e. moving backwards)
     */

    void motorB(int speed);
    /*  Motor B starts to move at the specified speed.
     *  
     *  @PARAM speed : Integer between -255 and 255 inclusive
     *    (Negative speeds represent the reverse direction
     *    i.e. moving backwards)
     */
     
    void motorC(int speed);
    /*  Motor C starts to move at the specified speed.
     *  
     *  @PARAM speed : Integer between -255 and 255 inclusive
     *    (Negative speeds represent the reverse direction
     *    i.e. moving backwards)
     */

    void motorD(int speed);
    /*  Motor D starts to move at the specified speed.
     *  
     *  @PARAM speed : Integer between -255 and 255 inclusive
     *    (Negative speeds represent the reverse direction
     *    i.e. moving backwards)
     */
    
    void stopMotorA();

    void stopMotorB();

    void stopMotorC();

    void stopMotorD();

    void moveAtAngle(int angle, int speed);
    /*  Makes the drivebase move at a particular angle, without
     *  turning, at the specified speed.
     *  
     *  @PARAM angle : integer between 0 and 360 (degrees)
     *  @PARAM speeed : Integer between -255 and 255 inclusive
     *    (Negative speeds represent the reverse direction
     *    i.e. moving backwards)
     */
     
    double angleSensorReading();
    /*  Return a sensor reading depicting the current heading
     *  (Angle that the robot is pointing towards)
     */

    void accelSensorReading();
    /*  Return a sensor reading from the accelerometer.
     */

    String getVoiceCommand();
    /*  Listens for a voice command from the Android app, and returns a      
     *  String with that voice command. If no command is available, it 
     *  returns an empty string “”.
     */



    void lightSensorReading();
    /*  Return a light intensity sensor reading from the accelerometer.
     */

  private:
    int _PIN_A;   //PIN FOR MOTOR A.
    int _PIN_B;   //PIN FOR MOTOR B.
    int _PIN_C;   //PIN FOR MOTOR C.
    int _PIN_D;   //PIN FOR MOTOR D.
    int _PIN_ACC; //PIN FOR IMU MODULE.

    Adafruit_DCMotor * _MotorA;
    Adafruit_DCMotor * _MotorB;
    Adafruit_DCMotor * _MotorC;
    Adafruit_DCMotor * _MotorD;

    Adafruit_LSM303_Accel_Unified accel;
    Adafruit_LSM303_Mag_Unified   mag;
    Adafruit_L3GD20_Unified       gyro;
    Adafruit_9DOF                 dof;

    Adafruit_MotorShield AFMS;
};

#endif
