/*
 * DrivingChassis.cpp
 *
 *  Created on: Jan 31, 2019
 *      Author: hephaestus
 *      Author: Peter Nikopoulos
 */

#include "DrivingChassis.h"
#include "Pose.h"

/**
 * Compute a delta in wheel angle to traverse a specific distance
 *
 * arc length	=	2*	Pi*	R*	(C/360)
 *
 * C  is the central angle of the arc in degrees
 * R  is the radius of the arc
 * Pi  is Pi
 *
 * @param distance a distance for this wheel to travel in MM
 * @return the wheel angle delta in degrees
 */
float DrivingChassis::distanceToWheelAngle(float distance) {
	return 0;
}

/**
 * Compute the arch length distance the wheel needs to travel through to rotate the base
 * through a given number of degrees.
 *
 * arc length	=	2*	Pi*	R*	(C/360)
 *
 * C  is the central angle of the arc in degrees
 * R  is the radius of the arc
 * Pi  is Pi
 *
 * @param angle is the angle the base should be rotated by
 * @return is the linear distance the wheel needs to travel given the this CHassis's wheel track
 */
float DrivingChassis::chassisRotationToWheelDistance(float angle) {
	return 0;
}

DrivingChassis::~DrivingChassis() {
	// do nothing
}

/**
 * DrivingChassis encapsulates a 2 wheel differential steered chassis that drives around
 *
 * @param left the left motor
 * @param right the right motor
 * @param wheelTrackMM is the measurment in milimeters of the distance from the left wheel contact point to the right wheels contact point
 * @param wheelRadiusMM is the measurment in milimeters of the radius of the wheels
 * @param imu The object that is used to access the IMU data
 *
 */
DrivingChassis::DrivingChassis(PIDMotor * left, PIDMotor * right,
		float wheelTrackMM, float wheelRadiusMM , GetIMU * imu) : robotPose(0,0,0)  //starts with pose of 0,0,0

{
	Serial.println("DrivingChassis::DrivingChassis constructor called here ");
	myleft = left;
	myright = right;
	robotPose.wheelTrackMM = wheelTrackMM;
	robotPose.wheelRadiusMM = wheelRadiusMM;
	IMU = imu;


}

/**
 * Start a drive forward action
 *
 * @param mmDistanceFromCurrent is the distance the mobile base should drive forward
 * @param msDuration is the time in miliseconds that the drive action should take
 *
 * @note this function is fast-return and should not block
 * @note pidmotorInstance->overrideCurrentPosition(0); can be used to "zero out" the motor to
 * 		 allow for relative moves. Otherwise the motor is always in ABSOLUTE mode
 */
void DrivingChassis::driveForward(float mmDistanceFromCurrent, int msDuration) {
}

/**
 * Start a turn action
 *
 * This action rotates the robot around the center line made up by the contact points of the left and right wheels.
 * Positive angles should rotate to the left
 *
 * This rotation is a positive rotation about the Z axis of the robot.
 *
 * @param degreesToRotateBase the number of degrees to rotate
 * @param msDuration is the time in miliseconds that the drive action should take
 *
 *  @note this function is fast-return and should not block
 *  @note pidmotorInstance->overrideCurrentPosition(0); can be used to "zero out" the motor to
 * 		  allow for relative moves. Otherwise the motor is always in ABSOLUTE mode
 */
void DrivingChassis::turnDegrees(float degreesToRotateBase, int msDuration) {

}

/**
 * Check to see if the chassis is performing an action
 *
 * @return false is the chassis is driving, true is the chassis msDuration has elapsed
 *
 * @note this function is fast-return and should not block
 */
bool DrivingChassis::isChassisDoneDriving() {
	return false;
}
/**
 * loop()
 *
 * a fast loop function that will update states of the motors based on the information from the
 * imu.
 */
void DrivingChassis::loop(){//polls for data every 20ms

	if(trigger){
		if(abs(IMU->getEULER_azimuth()) > 45){
			offset = IMU->getEULER_azimuth() * (-1);
			trigger = false;
		}
	}
	if (!loopFlag) {
		now = millis();
		loopFlag = true;
	}
	else {
		if(now + 20 <= millis()) {
			//Serial.println("looping");
			updatePose();
			loopFlag = false;
		}
	}

}

void DrivingChassis::updatePose(){
	angleRightMotor = myright->getAngleDegrees();  //gets angle from right and left motor
	angleLeftMotor = myleft->getAngleDegrees();
	IMUheading =  IMU->getEULER_azimuth() + offset;  //IMU mounted in reverse, going straight will give us heading of -180 unless we add 180 to offset system by PI
	double timestamp = micros();  //set in micros, if set in millis, timestamp will be 0
	robotPose.updateEncoderPositions(timestamp, angleRightMotor, angleLeftMotor, IMUheading);  //updates encoder position -> see Pose.cpp
	//robotPose.updateRobotCoordinates(angleRightMotor, angleLeftMotor, IMUheading);
}

void DrivingChassis::turn(double deg, double Kp) {
	deg = deg * (PI/180);
	//WITHOUT COMPLEMENTARY FILTER
	//double headingError = this->robotPose.theta - targetHeading;  //robotPose heading - target Heading  -1 because counterclockwise is negative in our coordinate system
	//JUST IMU
	// double headingError = ((offset + this->IMU->getEULER_azimuth()) * (PI/180)) - targetHeading ;
	//WITH COMPLEMENTARY FILTER
	double headingError = (((offset + this->IMU->getEULER_azimuth()) * (PI/180)) * .98 + this->robotPose.theta * .02) - deg;

	double effort = Kp * headingError;
	if(effort > 50) {
		effort = 50;
	}
	else if (effort < -50) {
		effort = -50;
	}
	this->myleft->setVelocityDegreesPerSecond(- effort);
	this->myright->setVelocityDegreesPerSecond(- effort);
}

bool DrivingChassis::turnTo(double deg){
	static bool trigger = true;
	if(trigger){

	}
	deg = deg * (PI/180);
	double headingError = (((this->robotPose.IMUheadingModulo) * (PI/180)) * 0.85 + this->robotPose.theta * 0.15) - deg; // IMU MOD ++++++++++++++++++++++++++++++++++++++++++++++++

	double effort = 25 * headingError; // 25 is KP
	if(effort > 50) {
		effort = 50;
	}
	else if (effort < -50) {
		effort = -50;
	} else if(effort > -1 && effort < 1){
		return true;
	}
	this->myleft->setVelocityDegreesPerSecond(- effort * 1.20);
	this->myright->setVelocityDegreesPerSecond(- effort);
	return false;
}


void DrivingChassis::driveStraight(double speed, double targetHeading, int Kp){ // usually 25 for point turn, usually 50 for driving
	targetHeading = targetHeading * (PI/180);
	//WITHOUT COMPLEMENTARY FILTER
	//double headingError = this->robotPose.theta - targetHeading;  //robotPose heading - target Heading  -1 because counterclockwise is negative in our coordinate system
	//JUST IMU
	// double headingError = ((offset + this->IMU->getEULER_azimuth()) * (PI/180)) - targetHeading ;
	//WITH COMPLEMENTARY FILTER
	double headingError = (((this->robotPose.IMUheadingModulo) * (PI/180)) * 0.98 + this->robotPose.theta * 0.02) - targetHeading; // IMU MOD+++++++++++++++++++++++++++++++++++++++++
	double effort = Kp * headingError;
	this->myleft->setVelocityDegreesPerSecond(speed - effort);
	this->myright->setVelocityDegreesPerSecond(-speed - effort);
}


double DrivingChassis::mmTOdeg(double mm){
	return (mm/(wheelRadius * (2*PI))) * 360;
}


bool DrivingChassis::turn90CCW(){
	static bool trigger = true;
	if(trigger){
		trigger = false;
		dir = robotPose.returnRobotHeading(this->IMU->getEULER_azimuth()) + 90;
	}
	if(turnTo(dir)){
		trigger = true;
		return true;
	}
	return false;
}

bool DrivingChassis::turn90CW(){
	static bool trigger = true;
	if(trigger){
		trigger = false;
		dir = robotPose.returnRobotHeading(this->IMU->getEULER_azimuth()) - 90;
	}
	if(turnTo(dir)){
		trigger = true;
		return true;
	}
	return false;
}

bool DrivingChassis::driveTo(int Xcord, int Ycord){
	static bool trigger = true;
	if(trigger){
		trigger = false;
		//Generating a path
		Xdist = Xcord - this->robotPose.posX;
		if(Xdist != 0){
			if(Xcord < this->robotPose.posX){
				heading1 = -90;
			}
			if(Xcord > this->robotPose.posX){
				heading1 = 90;
			}
		}
		Ydist = Ycord - this->robotPose.posY;
		if(Ydist != 0){
			if(Ycord < this->robotPose.posY){
				heading2 = 0;
			}
			if(Ycord > this->robotPose.posY){
				heading2 = 180;
			}
		} else {
			heading2 = heading1;
		}

	}

	switch(Step){
	case toHeading1:
		Serial.println("H1");
		if(turnTo(heading1)){
			Step = driveX;
		}
		break;
	case driveX:
		Serial.println("DX");
		this->driveStraight(200, heading1, 1000);
		if(this->robotPose.posX == Xcord){
			Step = toHeading2;
		}
		break;
	case toHeading2:
		Serial.println("H2");
		if(turnTo(heading2)){
			Step = driveY;
		}
		break;
	case driveY:
		Serial.println("DY");
		this->driveStraight(200, heading2, 1000);
		if(this->robotPose.posY == Ycord){
			Step = done;
		}
		break;
	case done:
		Serial.println("DONE");
		trigger = true;
		Step = toHeading1;
		return true;
		break;
	}

	return false;
}

int DrivingChassis::getOrientation(int building, int row){
	if(this->robotPose.posX == building){
		if(this->robotPose.posY == row - 1){
			return 4;
		}else{
			return 2;
		}
	}
	if(this->robotPose.posY == row){
		if(this->robotPose.posX == building - 1){
			return 3;
		}else{
			return 1;
		}
	}
	return 0;
}

bool DrivingChassis::turnTheCorner(bool CCW){
	switch(corner){
	case drive:
		//Serial.println("TURNTHECORNER DRIVING");
		//Serial.println(driveCount);
		if(driveOneBlock()){
			driveCount++;
			corner = turning;
		}
		break;

	case turning:
	//	Serial.println("TURNTHECORNER TURNING");
	//	Serial.println(driveCount);
		if(driveCount < 2 && CCW){
			if(turn90CCW()){
				corner = drive;
			}
		}else if(driveCount < 2 && !CCW){
			if(turn90CW()){
				corner = drive;
			}
		}else{
			driveCount = 0;
			corner = drive;
			return true;
		}
		break;
	}

	return false;

}

bool DrivingChassis::driveOneBlock(){
	static bool trigger = true;
	if(trigger){
		trigger = false;
		startingHeading = this->robotPose.IMUheadingModulo;
		x = robotPose.posX;
		y = robotPose.posY;
	}
	//Serial.println(startingHeading);
	//Serial.println(trigger);
	//Serial.println("driving ONEEE");
	driveStraight(200, startingHeading, 1000);
	if(abs(robotPose.posX - x) >= 1 || abs(robotPose.posY - y) >= 1){
		trigger = true;
		return true;
	}
	return false;
}

bool DrivingChassis::driveDistanceBlocks(double fractionBlock){
	static bool trigger = true;
	if(trigger){
		trigger = false;
		startingHeading = this->robotPose.IMUheadingModulo;
		x = robotPose.posX;
		y = robotPose.posY;
	}
	//Serial.println(startingHeading);
	//Serial.println(trigger);
	//Serial.println("driving ONEEE");
	driveStraight(200, startingHeading, 1000);
	if(abs(robotPose.posX - x) >= fractionBlock || abs(robotPose.posY - y) >= fractionBlock){
		trigger = true;
		return true;
	}
	return false;
}






bool DrivingChassis::distanceDrive (double mm){ // Useless with implementation of PlotDrive
	double target = mmTOdeg(mm);
	distanceError =  abs(this->myright->getAngleDegrees()) - target;
	double effort = kpDistance * distanceError;
	this->driveStraight(-effort, 0, 1000);
	if(effort < 10 && effort > -10){ // might need some tweaking
		return true;
	}else{
		return false;
	}
}

//bool DrivingChassis::drivePlotsinDir (double plots, double heading, bool isX){
//	static bool trigger =  true;
//	if(trigger){
//		trigger = false;
//		changeInXpos = this->robotPose.posX;
//		changeInYpos = this->robotPose.posY;
//		//targetDistance = (abs(this->myright->getAngleDegrees()) + abs(this->myleft->getAngleDegrees()))*0.5 + abs(plots) * plotLen;
//	}
//	Serial.println("===========================================");
//	Serial.println(changeInXpos);
//	double effort = 200;
//
//
//	if(isX){
//		changeInXpos = abs(changeInXpos - this->robotPose.posX);
//		if( abs(Xdist) == changeInXpos){ // distanceTraveled >= targetDistance
//				trigger = true;
//				this->driveStraight(0, heading, 0);
//				return true;
//			}else{
//				this->driveStraight(effort, heading, 1000);
//				return false;
//			}
//	}else{
//		changeInYpos = abs(changeInYpos - this->robotPose.posY);
//		if( abs(Ydist) == changeInYpos){ // distanceTraveled >= targetDistance
//				trigger = true;
//				this->driveStraight(0, heading, 0);
//				return true;
//			}else{
//				this->driveStraight(effort, heading, 1000);
//				return false;
//			}
//	}
//}
//
//
//bool DrivingChassis::turnDrive(double deg){
//	this->turn(deg,25);
//	if(this->IMU->getEULER_azimuth() < deg + 1 && this->IMU->getEULER_azimuth() > deg - 1){ //Possibly make this a range so robot gets out of turn drive function faster
//		return true;
//	}else{
//		return false;
//	}
//}
//
//bool DrivingChassis::turnDriveEffortBool(double deg){
//	deg = deg * (PI/180);
//
//	double headingError = (((offset + this->IMU->getEULER_azimuth()) * (PI/180)) * .98 + this->robotPose.theta * .02) - deg;
//	double effort = 25 * headingError;
//	if(effort > 50) {
//		effort = 50;
//	}
//	else if (effort < -50) {
//		effort = -50;
//	}else if(effort > -1 && effort < 1){
//		return true;
//	}
//	Serial.println(effort);
//	this->myleft->setVelocityDegreesPerSecond(- effort);
//	this->myright->setVelocityDegreesPerSecond(- effort);
//	return false;
//}













void DrivingChassis::printTemporaryBuildingArray(){
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			Serial.print(String(buildingArray[i][j]));
		}
		printf("\n");
	}
}

