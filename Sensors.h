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
#include "Pose.h"

class Sensors {
public:
	Sensors();
	int attach(int trig, int echo);
	double PingUltrasonic();
	boolean foundRoadBlockInStreet();

private:
	int triggerPin = 0;
	int echoPin = 0;
	int inputSignal = 10; //us
	int convert = 1/58; //cm/us
	long dTime = 0;
	bool reading = false;
	bool trigger = true;
};

#endif /* SENSORS_H_ */
