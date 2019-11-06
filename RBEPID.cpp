/*
 * Messages.h
 *
 *  Created on: 10/1/16
 *      Author: joest
 */
#include "Arduino.h"
#include "RBEPID.h"

//Class constructor
RBEPID::RBEPID() {

}

//Function to set PID gain values
void RBEPID::setpid(float P, float I, float D) {
	kp = P;
	ki = I;
	kd = D;
	errorSum = 0;
}

/**
 * calc the PID control signel
 *
 * @param setPoint is the setpoint of the PID system
 * @param curPosition the current position of the plan
 * @return a value from -1.0 to 1.0 representing the PID control signel
 */
float RBEPID::calc(double setPoint, double curPosition) {

	// calculate error
	float err = setPoint - curPosition;
	// calculate derivative of error
	//TODO
	// calculate integral error. Running average is best but hard to implement

	/*
	this ->errorArray[this->errorIndex] = err;

	this->errorSum = 0;
	for(int i = 0; i <= 50; i++){
		this->errorSum += errorArray[i];
	}

	this->errorIndex++;

	if(this->errorIndex > 50){
		this->errorIndex = 0;
	}

	float Ierr = (this->errorSum/50);
	*/
	errorSum += err;
	errorIndex++;
	Ierr += (this->errorSum/ errorIndex);
	// sum up the error value to send to the motor based off gain values.
	//TODO

	float out = err * kp + errorSum *ki;
	// simple P controller
	//return the control signal from -1 to 1
	if (out > 1)
		out = 1;
	if (out < -1)
		out = -1;
	return out ;
}

/**
 * Clear the internal representation fo the integral term.
 *
 */
void RBEPID::clearIntegralBuffer() {
	for(int j = 0; j < 50; j++){
		this->errorArray[j] = 0;
	}
}
