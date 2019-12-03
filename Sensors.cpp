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
	this->sigPin = -1;

}

int Sensors::attach(int trig, int echo){
	this->triggerPin = trig;
	this->echoPin = echo;
	pinMode(this->triggerPin, OUTPUT);
	digitalWrite(this->triggerPin, LOW);
	pinMode(this->echoPin, INPUT);
	return 0;
}

int Sensors::attach(int sig){
	this->sigPin = sig;
	pinMode(this->sigPin, OUTPUT);
	digitalWrite(this->sigPin, LOW);
	return 0;
}

double Sensors::PingUltrasonic3Pin(){
	if(!reading3Pin){
		if(trigger3Pin){
			dTime3Pin = micros();
			digitalWrite(triggerPin, HIGH);
			trigger3Pin = false;
		}
		if(!trigger3Pin && dTime3Pin + 20 < micros()){
			digitalWrite(triggerPin, LOW);
			reading3Pin = false;
			trigger3Pin = true;
			double pulseWidth = pulseIn(echoPin,HIGH);
			double distance = pulseWidth * (1/58.0) * 10;
			return distance;  //pulsewidth of echoPin multiplied by conversion factor converted to mm from cm
		}
	}
	return -1.0;
}

double Sensors::PingUltrasonic4Pin(){
	if(!reading4Pin){
		if(trigger4Pin){
			dTime4Pin = micros();
			pinMode(this->sigPin, OUTPUT);
			digitalWrite(sigPin, HIGH);
			trigger4Pin = false;
		}
		if(!trigger4Pin && dTime4Pin + 20 < micros()){
			digitalWrite(triggerPin, LOW);
			reading3Pin = false;
			trigger3Pin = true;
			double pulseWidth = pulseIn(echoPin,HIGH);
			double distance = pulseWidth * (1/58.0) * 10;
			return distance;  //pulsewidth of echoPin multiplied by conversion factor converted to mm from cm
		}

	}

}




