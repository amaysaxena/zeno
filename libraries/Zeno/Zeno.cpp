/*
  # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
  #									#									
  #	    Zeno.cpp - Library for the Zeno educational		        #
  #	    robotics platform.					        #							
  #	    Read included example files in the /examples directory      #
  #         for sample usage.					        #							
  #									#									
  #         Created by Amay S. Saxena, March 31, 2017.		        #				
  #	    Released into the public domain.			        #					
  #									#																						#
  #									#									
  # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
*/

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

	Zeno :: Zeno(bool MotorDPresent) {
		/*  Initializes the basic Zeno drivebase, setting the 
	     *  drive motor and sensor pins to the appropriate values.
	     *
	     *  @PARAM MotorDPresent : boolean- set this to true if you have
	     *		included a fourth motor in your project 
	     *		(apart from the 3 drivebase motors).
	     */

		// Create the motor shield object with the default I2C address
		AFMS = Adafruit_MotorShield();
		// Or, create it with a different I2C address (say for stacking)
		// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

		// Initialize the four ports, M1, M2, M3 and M4.
		Adafruit_DCMotor *MotorA1 = AFMS.getMotor(1);
		Adafruit_DCMotor *MotorB1 = AFMS.getMotor(2);
		Adafruit_DCMotor *MotorC1 = AFMS.getMotor(3);

		accel = Adafruit_LSM303_Accel_Unified(30301);
		mag   = Adafruit_LSM303_Mag_Unified(30302);
		gyro  = Adafruit_L3GD20_Unified(20);
		dof   = Adafruit_9DOF();

		if (MotorDPresent) {
			Adafruit_DCMotor *MotorD1 = AFMS.getMotor(4);
			_MotorD = MotorD1;
		}

		_MotorA = MotorA1;
		_MotorB = MotorB1;
		_MotorC = MotorC1;
		
	}

	void Zeno :: ZenoSetup() {
		 //Serial.begin(9600);           // set up Serial library at 9600 bps

	     AFMS.begin();  // create with the default frequency 1.6KHz
	     //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
		  
	     // Set the speed to start, from 0 (off) to 255 (max speed)

	    if(!accel.begin()) {
		    /* There was a problem detecting the ADXL345 ... check your connections */
		    Serial.print("Ooops, no sensor detected! Check your wiring!");
		    while(1);
		}
		
		if(!mag.begin()) {
		    /* There was a problem detecting the LSM303 ... check your connections */
		    Serial.print("Ooops, no sensor detected! Check your wiring!");
		    while(1);
		}

		if(!gyro.begin()) {
		    /* There was a problem detecting the L3GD20 ... check your connections */
		    Serial.print("Ooops, no sensor detected! Check your wiring!");
		    while(1);
		}

	     _MotorA->setSpeed(150);
	     _MotorB->setSpeed(150);
	     _MotorC->setSpeed(150);
	     _MotorD->setSpeed(150);

	     _MotorA->run(FORWARD);
	     _MotorB->run(FORWARD);
	     _MotorC->run(FORWARD);
	     _MotorD->run(FORWARD);
	     // turn on motors

	     _MotorA->run(BACKWARD);
	     _MotorB->run(BACKWARD);
	     _MotorC->run(BACKWARD);
	     _MotorD->run(BACKWARD);
	}
    
    void Zeno :: moveForward(int speed) {
	    /* The robot begins mooving forward.
	     *  
	     *  @PARAM speed : Integer between -255 and 255 inclusive
	     *    (Negative speeds represent the reverse direction
	     *    i.e. moving backwards)
	     */

    	if (speed > 0) {
	    	_MotorA->setSpeed(speed);
	    	_MotorA->run(FORWARD);

	    	_MotorB->setSpeed(speed);
	    	_MotorB->run(FORWARD);

    	} else if (speed == 0) {
    		stopMoving();

    	} else {
	    	_MotorA->setSpeed(abs(speed));
	    	_MotorA->run(BACKWARD);

	    	_MotorB->setSpeed(abs(speed));
	    	_MotorB->run(FORWARD);
    	}

    }

    void Zeno :: stopMoving() {
	     /* Stops all robot drivebase motion.
	      */
    	stopMotorA();
    	stopMotorB();
    	stopMotorC();
    }

    void Zeno :: motorA(int speed) {
	    /*  Motor A starts to move at the specified speed.
	     *  
	     *  @PARAM speed : Integer between -255 and 255 inclusive
	     *    (Negative speeds represent the reverse direction
	     *    i.e. moving backwards)
	     */
    	if (speed > 0) {
	    	_MotorA->setSpeed(speed);
	    	_MotorA->run(FORWARD);

    	} else if (speed == 0) {
    		stopMotorA();

    	} else {
	    	_MotorA->setSpeed(abs(speed));
	    	_MotorA->run(BACKWARD);
    	}

    }

    void Zeno :: motorB(int speed) {
	    /*  Motor B starts to move at the specified speed.
	     *  
	     *  @PARAM speed : Integer between -255 and 255 inclusive
	     *    (Negative speeds represent the reverse direction
	     *    i.e. moving backwards)
	     */

    	if (speed > 0) {
	    	_MotorB->setSpeed(speed);
	    	_MotorB->run(FORWARD);

    	} else if (speed == 0) {
    		stopMotorB();

    	} else {
	    	_MotorB->setSpeed(abs(speed));
	    	_MotorB->run(BACKWARD);
    	}
    }
     
    void Zeno :: motorC(int speed) {
	    /*  Motor C starts to move at the specified speed.
	     *  
	     *  @PARAM speed : Integer between -255 and 255 inclusive
	     *    (Negative speeds represent the reverse direction
	     *    i.e. moving backwards)
	     */
    	if (speed > 0) {
	    	_MotorC->setSpeed(speed);
	    	_MotorC->run(FORWARD);

    	} else if (speed == 0) {
    		stopMotorC();

    	} else {
	    	_MotorC->setSpeed(abs(speed));
	    	_MotorC->run(BACKWARD);
    	}
    }

    void Zeno :: motorD(int speed) {
	    /*  Motor D starts to move at the specified speed.
	     *  
	     *  @PARAM speed : Integer between -255 and 255 inclusive
	     *    (Negative speeds represent the reverse direction
	     *    i.e. moving backwards)
	     */

    	if (speed > 0) {
	    	_MotorD->setSpeed(speed);
	    	_MotorD->run(FORWARD);

    	} else if (speed == 0) {
    		stopMotorD();

    	} else {
	    	_MotorD->setSpeed(abs(speed));
	    	_MotorD->run(BACKWARD);
    	}
    }

    void Zeno :: stopMotorA() {
    	/* Stop Motor A.
    	*/

    	_MotorA->run(RELEASE);
    }

    void Zeno :: stopMotorB() {
    	/* Stop Motor B.
    	*/

    	_MotorB->run(RELEASE);
    }

    void Zeno :: stopMotorC() {
    	/* Stop Motor C.
    	*/

    	_MotorC->run(RELEASE);
    }

    void Zeno :: stopMotorD() {
    	/* Stop Motor D.
    	*/

    	_MotorD->run(RELEASE);
    }
    
    void Zeno :: moveAtAngle(int angle, int speed) {
	    /*  Makes the drivebase move at a particular angle, without
	     *  turning, at the specified speed.
	     *  
	     *  @PARAM angle : integer between 0 and 360 (degrees)
	     *  @PARAM speeed : Integer between -255 and 255 inclusive
	     *    (Negative speeds represent the reverse direction
	     *    i.e. moving backwards)
	     */
    	int speedA = round(speed * cos((150 - angle) * (M_PI / 180)));
    	int speedB = round(speed * cos((30 - angle) * (M_PI / 180)));
    	int speedC = round(speed * cos((270 - angle) * (M_PI / 180)));

    	motorA(speedA);
    	motorB(speedB);
    	motorC(speedC);

    	//HAVE NOT IMPLEMENTED CLOSED LOOP CONTROL YET!!!!!!!!!!!!!!!!!!!!!!!

    }
     
    double Zeno :: angleSensorReading() {
    /*  Return a sensor reading depicting the current heading
     *  (Angle that the robot is pointing towards)
     */
    	sensors_event_t gyro_event;
    	sensors_event_t mag_event;

    	gyro.getEvent(&gyro_event);
    	mag.getEvent(&mag_event);

    	sensors_vec_t   orientation;
		mag.getEvent(&mag_event);

	 	  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation)) {
		    /* 'orientation' should have valid .heading data now */
		    	return orientation.heading;

			    // Serial.print(F("Heading: "));
		    	// Serial.print(orientation.heading);
		    	// Serial.print(F("; "));
		  }
    }

    String Zeno :: getVoiceCommand() {
	    Serial.begin(9600);
	    String readString = "";

	    if(!Serial.available() > 0) {
	    	return "";
	  	}
  
	  while (Serial.available() > 0) {
		    delay(3);
		    char c = Serial.read(); 
		    readString += c;
	  }
	  return readString;
	}


