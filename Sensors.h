/*
 * Sensors.h
 *
 *  Created on: Nov 21, 2019
 *      Author: Peter Nikopoulos
 */

#ifndef SENSORS_H_
#define SENSORS_H_
#include "config.h"
#include "Arduino.h"

class Sensors {
public:
	Sensors();
	int attach(int trig, int echo);
	int attach(int sig);
	double PingUltrasonic3Pin();
	double PingUltrasonic4Pin();


private:
	int triggerPin = 0;
	int echoPin = 0;
	int sigPin = 0;
	int inputSignal = 10; //us
	int convert = 1/58; //cm/us
	long dTime3Pin = 0;
	long dTime4Pin = 0;
	bool reading3Pin = false;
	bool trigger3Pin = true;
	bool reading4Pin = false;
	bool trigger4Pin = true;
};

#endif /* SENSORS_H_ */
