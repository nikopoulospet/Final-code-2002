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
	digitalWrite(this->triggerPin, LOW);
	pinMode(this->echoPin, INPUT);
	return 0;
}

double Sensors::PingUltrasonic(){
	if(!reading){
		if(trigger){
			dTime = micros() - dTime;
			digitalWrite(this->triggerPin, HIGH);
			Serial.print("++++++++++++++++++++++++++++++++");
			Serial.println(dTime);
			trigger = false;
		}
		if(!trigger && dTime + 20 <= micros() - dTime){
			digitalWrite(this->triggerPin, LOW);
			reading = true;
			trigger = true;
			Serial.print("===================================");
		}
	}
	if(digitalRead(this->echoPin) == HIGH){
		reading = false;
		Serial.println("In output");
		//double duration = pulseIn(this->echoPin, HIGH);

	}
	return 0;
}




