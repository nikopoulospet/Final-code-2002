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
	//digitalWrite(triggerPin, LOW);
	if(!reading){
		//Serial.print("Hello");
		if(trigger){
			dTime = micros();
			digitalWrite(triggerPin, HIGH);
			//Serial.print("Hiya");
			//Serial.print("++++++++++++++++++++++++++++++++");
			//Serial.println(dTime);
			trigger = false;
		}
		if(!trigger && dTime + 20 < micros()){
			digitalWrite(triggerPin, LOW);
			reading = false;
			trigger = true;
		//	Serial.println("Please help me");
			return pulseIn(echoPin,HIGH);

			//Serial.print("===================================");
		}
	}
	return -1.0;
}




