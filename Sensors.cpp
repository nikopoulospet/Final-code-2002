/*
 * Sensors.cpp
 *
 *  Created on: Nov 21, 2019
 *      Author: Peter Nikopoulos
 */

#include "Sensors.h"

Sensors::Sensors() {
	this->triggerPin = -1;
	this->echoPin = -1;

}

int Sensors::attach(int trig, int echo){
	this->triggerPin = trig;
	this->echoPin = echo;
	pinMode(this->triggerPin, OUTPUT);
	digitalWrite(this->triggerPin, LOW);  //must write trigger pin to low at the attach function
	pinMode(this->echoPin, INPUT);
	return 0;
}

double Sensors::PingUltrasonic(){
	if(!reading){ //if we have not taken a reading yet,
		if(trigger){  //write trigger pin to high and do not enter this loop until a reading has been taken
			dTime = micros();
			digitalWrite(triggerPin, HIGH);
			trigger = false;
		}
		if(!trigger && dTime + 20 < micros()){ //if 20 microseconds have gone by (datasheet value), set triggerpin to low and measure the pulsewidth of the echo pin
			digitalWrite(triggerPin, LOW);
			reading = false;
			trigger = true;
			double pulseWidth = pulseIn(echoPin,HIGH);
			double distance = pulseWidth * (1/58.0) * 10;
			return distance;  //pulsewidth of echoPin multiplied by conversion factor converted to mm from cm

		}
	}
	return -1.0;
}




