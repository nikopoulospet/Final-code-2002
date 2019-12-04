/*
 * Pose.h
 *
 *  Created on: Nov 14, 2019
 *      Author: Brian
 */

#ifndef POSE_H_
#define POSE_H_
#include <math.h>
#include <Arduino.h>

class Pose {
private:
	/*PIDMotor * leftMotor;
	PIDMotor * rightMotor;
	//GetIMU * IMU;
	 */
	bool loopFlag = false;
	double now = 0;





public:
	Pose(double x, double y, double theta);

	double lastEncoder0 = 0; //left
	double lastEncoder1 = 0;
	double lastTimestamp = -1;
	double lastIMUHeading = 0;
	float wheelTrackMM = 225;
	float wheelRadiusMM = 25.6;
	double x = 0;
	double y = 0;
	double theta = 0;
	int posX = 0;
	int posY = 5; // Starting position for robot
	double avgTraveled = 0.0;
	double blockLeninDeg = 914 / 2; // 3:1 ???
	int IMUheadingModulo = 0;
	double prevIMUheading;

	void updateEncoderPositions(double timestamp, double encoder0, double encoder1, double IMUheading);
	void setRobotPosition(int posX, int posY);
};


#endif /* POSE_H_ */
