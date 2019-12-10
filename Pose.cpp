/*
 * Pose.cpp
 *
 *  Created on: Nov 14, 2019
 *      Author: Brian
 */
#include "Pose.h"

Pose::Pose(double x, double y, double theta) {
	this->x = x;
	this->y = y;
	this->theta = theta;
}

void Pose:: updateEncoderPositions(double timestamp, double encoder0, double encoder1, double IMUheading){  //same code from java, transposed into eclipse second order forward kinematic model
	if (lastTimestamp > 0) {
		double deltaTime = timestamp - lastTimestamp;
		deltaEncoder0 = encoder0 - lastEncoder0;
		deltaEncoder1 = encoder1 - lastEncoder1;
		double deltaIMU = IMUheading - lastIMUHeading;
		if (abs(deltaEncoder1) > 0 && abs(deltaEncoder0) > 0) {
			//Serial.println("Delta Time=" + String(deltaTime) + " enc0=" + String(deltaEncoder0) + " enc1=" + String(deltaEncoder1)+ " heading=" + String(deltaIMU));
		}

		// do the Forward Kinematic update

		double speedofLeftWheel = (-deltaEncoder1 / deltaTime) * (PI / 180) * wheelRadiusMM; //   m/s  -1 on deltaEncoder 1 may be changed after negative issue is resolved
		double speedofRightWheel = (deltaEncoder0 / deltaTime) * (PI / 180) * wheelRadiusMM;
		double initialVelocity = (speedofLeftWheel + speedofRightWheel)/2;
		double angularVelocity = (speedofRightWheel - speedofLeftWheel) / wheelTrackMM;
		double nextAngularPosition = (angularVelocity * deltaTime) + theta;


		double changeX = deltaTime * initialVelocity * cos((theta + nextAngularPosition)/2);
		x = x + changeX;
		double changeY = deltaTime * initialVelocity * sin((theta + nextAngularPosition)/2);
		y = y + changeY;

		theta = nextAngularPosition;


		// adding block counting / robot position update. checks heading and distance traveled to figure out robot X and Y pos
		IMUheadingModulo = int(IMUheading) % 360;
		if((deltaEncoder0 > 0 && deltaEncoder1 < 0)||(deltaEncoder0 < 0 && deltaEncoder1 > 0)){ // if robot is traveling straight(ish)
			avgTraveled += abs(deltaEncoder1); //(abs(deltaEncoder0) + abs(deltaEncoder1)) * 0.5;   // deltaEnc 0 and deltaEnc 1 start at the same value and slowly appear to drift apart. this could be a PID or motor issue where the motors need to spin at different speeds in order to maintian a constant heading.
			// the working solution at the moment is to only track one delta Encoder. this means the slight dift apart of both encoder values wont slowly change the distance needed to travel for each block. a magic value still needs to be found, indicating that issues remain but those are unresolved for now. this fix should let us drive a constant distance for one block.
			//Serial.println(String(avgTraveled) + "=====================================================");
			if (avgTraveled >= blockLeninDeg){
				avgTraveled = 0;
				if((abs(IMUheadingModulo) <= 360 && abs(IMUheadingModulo) >= 340) || (abs(IMUheadingModulo) >= 0 && abs(IMUheadingModulo) <= 20)){//IMU is in degrees
					setRobotPosition(posX, posY - 1);
					prevIMUheading = 1;
				}

				if((abs(IMUheadingModulo) <= 110 && abs(IMUheadingModulo) >= 70)){//IMU is in degrees
					if(IMUheadingModulo > 0){
						setRobotPosition(posX + 1, posY);
						prevIMUheading = 2;
					}else{
						setRobotPosition(posX - 1, posY);
						prevIMUheading = 4;
					}
				}

				if((abs(IMUheadingModulo) <= 200 && abs(IMUheadingModulo) >= 160)){
					setRobotPosition(posX, posY + 1);
					prevIMUheading = 3;
				}

				if((abs(IMUheadingModulo) >= 250 && abs(IMUheadingModulo) <= 290)){//IMU is in degrees
					if(IMUheadingModulo > 0){
						setRobotPosition(posX - 1, posY);
						prevIMUheading = 4;
					}else{
						setRobotPosition(posX + 1, posY);
						prevIMUheading = 2;
					}
				}
			}
		}else{ //robot is turning
			avgTraveled = 0;
		}



	}
	Serial.println("PosX " + String(posX) + " PosY " + String(posY));

	Serial.println("EncO =" + String(deltaEncoder0) +"Enc1 =" + String(deltaEncoder1));
	//Serial.println(millis());
	Serial.println("Final pose x= " + String(x) + " y= " + String(y) + " theta= " + String(theta) + " IMU Heading: " + String(((IMUheading)))); //print of pose
	//Serial.println(millis());


	lastEncoder0 = encoder0;  //reset values for next loop through
	lastEncoder1 = encoder1;
	lastTimestamp = timestamp;
	lastIMUHeading = IMUheading;


}


void Pose::setRobotPosition(int posX, int posY){
	this->posX = posX;
	this->posY = posY;
}

int Pose::returnRobotHeading(double IMUheading){
	IMUheadingModulo = int(IMUheading) % 360;
	if((abs(IMUheadingModulo) <= 360 && abs(IMUheadingModulo) >= 340) || (abs(IMUheadingModulo) >= 0 && abs(IMUheadingModulo) <= 20)){//IMU is in degrees
		prevIMUheading = 1;
	}

	if((abs(IMUheadingModulo) <= 110 && abs(IMUheadingModulo) >= 70)){//IMU is in degrees
		if(IMUheadingModulo > 0){
			prevIMUheading = 2;
		}else{
			prevIMUheading = 4;
		}
	}

	if((abs(IMUheadingModulo) <= 200 && abs(IMUheadingModulo) >= 160)){
		prevIMUheading = 3;
	}


	if((abs(IMUheadingModulo) >= 250 && abs(IMUheadingModulo) <= 290)){//IMU is in degrees
		if(IMUheadingModulo > 0){
			prevIMUheading = 4;
		}else{
			prevIMUheading = 2;
		}
	}

	return prevIMUheading;

}

int Pose::lookingAtX(){
	if(returnRobotHeading(IMUheadingModulo) == 2){ // might be fucky because of loop updates
		return posX += 1;
	}else if(returnRobotHeading(IMUheadingModulo) == 4){
		return posX -= 1;
	} else {
		return posX;
	}
}

int Pose::lookingAtY(){
	if(returnRobotHeading(IMUheadingModulo) == 1){ // might be fucky because of loop updates
		return posY -= 1;
	}else if(returnRobotHeading(IMUheadingModulo) == 3){
		return posY += 1;
	} else {
		return posY;
	}
}






