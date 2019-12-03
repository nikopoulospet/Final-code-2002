/*
 * StudentsRobot.cpp
 *
 *  Created on: Dec 28, 2018
 *      Author: hephaestus
 */

#include "StudentsRobot.h"
#define wheelTrackMM  225   //pass in wheeltrack and wheel radius into mm
#define	wheelRadiusMM 25.6

StudentsRobot::StudentsRobot(PIDMotor * motor1, PIDMotor * motor2,
		PIDMotor * motor3, Servo * servo, IRCamSimplePacketComsServer * IRCam,
		GetIMU * imu) : ace(motor1,motor2,wheelTrackMM,wheelRadiusMM,imu), Ultrasonic1(), fieldMap()



{
	Serial.println("StudentsRobot::StudentsRobot constructor called here ");
	this->servo = servo;
	this->motor1 = motor1;
	this->motor2 = motor2;
	this->motor3 = motor3;
	IRCamera = IRCam;
	IMU = imu;
	//ace = new DrivingChassis(motor1,motor2,wheelTrackMM,wheelRadiusMM,imu);
#if defined(USE_IMU)
	IMU->setXPosition(200);
	IMU->setYPosition(0);
	IMU->setZPosition(0);
#endif
	// Set the PID Clock gating rate. The PID must be 10 times slower than the motors update rate
	motor1->myPID.sampleRateMs = 5; //
	motor2->myPID.sampleRateMs = 5; //
	motor3->myPID.sampleRateMs = 5;  // 10khz H-Bridge, 0.1ms update, 1 ms PID

	// Set default P.I.D gains
	motor1->myPID.setpid(0.00015, 0, 0);
	motor2->myPID.setpid(0.00015, 0, 0);
	motor3->myPID.setpid(0.00015, 0, 0);

	motor1->velocityPID.setpid(0.004, 0.00001, 0);
	motor2->velocityPID.setpid(0.004, 0.00001, 0);
	motor3->velocityPID.setpid(0.1, 0, 0);
	// compute ratios and bounding
	double motorToWheel = 3;
	motor1->setOutputBoundingValues(-255, //the minimum value that the output takes (Full reverse)
			255, //the maximum value the output takes (Full forward)
			0, //the value of the output to stop moving
			125, //a positive value subtracted from stop value to creep backward
			125, //a positive value added to the stop value to creep forwards
			16.0 * // Encoder CPR
			50.0 * // Motor Gear box ratio
			motorToWheel * // motor to wheel stage ratio
			(1.0 / 360.0) * // degrees per revolution
			2, // Number of edges that are used to increment the value
			480, // measured max degrees per second
			150 // the speed in degrees per second that the motor spins when the hardware output is at creep forwards
	);
	motor2->setOutputBoundingValues(-255, //the minimum value that the output takes (Full reverse)
			255, //the maximum value the output takes (Full forward)
			0, //the value of the output to stop moving
			125, //a positive value subtracted from stop value to creep backward
			125, //a positive value added to the stop value to creep forwards
			16.0 * // Encoder CPR
			50.0 * // Motor Gear box ratio
			motorToWheel * // motor to wheel stage ratio
			(1.0 / 360.0) * // degrees per revolution
			2, // Number of edges that are used to increment the value
			480, // measured max degrees per second
			150	// the speed in degrees per second that the motor spins when the hardware output is at creep forwards
	);
	motor3->setOutputBoundingValues(-255, //the minimum value that the output takes (Full reverse)
			255, //the maximum value the output takes (Full forward)
			0, //the value of the output to stop moving
			125, //a positive value subtracted from stop value to creep backward
			125, //a positive value added to the stop value to creep forwards
			16.0 * // Encoder CPR
			50.0 * // Motor Gear box ratio
			1.0 * // motor to arm stage ratio
			(1.0 / 360.0) * // degrees per revolution
			2, // Number of edges that are used to increment the value
			1400, // measured max degrees per second
			50 // the speed in degrees per second that the motor spins when the hardware output is at creep forwards
	);
	// Set up the Analog sensors
	pinMode(ANALOG_SENSE_ONE, ANALOG);
	pinMode(ANALOG_SENSE_TWO, ANALOG);
	pinMode(ANALOG_SENSE_THREE, ANALOG);
	pinMode(ANALOG_SENSE_FOUR, ANALOG);
	// H-Bridge enable pin
	pinMode(H_BRIDGE_ENABLE, OUTPUT);
	// Stepper pins
	//pinMode(STEPPER_DIRECTION, OUTPUT);
	//pinMode(STEPPER_STEP, OUTPUT);
	// User button
	pinMode(BOOT_FLAG_PIN, INPUT_PULLUP);
	//Test IO
	pinMode(WII_CONTROLLER_DETECT, OUTPUT);

	//SENSOR
	Ultrasonic1.attach(TrigPIN, EchoPIN);

}
/**
 * Seperate from running the motor control,
 * update the state machine for running the final project code here
 */
void StudentsRobot::updateStateMachine() {
	digitalWrite(WII_CONTROLLER_DETECT, 1);
	long now = millis();

	//polling for pose every 20ms, see DrivingChassis.cpp
	switch (status) {
	case StartupRobot:
		//Do this once at startup
		status = StartRunning;
		Serial.println("StudentsRobot::updateStateMachine StartupRobot here ");
		break;
	case StartRunning:
		Serial.println("Start Running");

		digitalWrite(H_BRIDGE_ENABLE, 1);
		// Start an interpolation of the motors
		motor1->startInterpolationDegrees(motor1->getAngleDegrees(), 1000, SIN);
		motor2->startInterpolationDegrees(motor2->getAngleDegrees(), 1000, SIN);
		motor3->startInterpolationDegrees(motor3->getAngleDegrees(), 1000, SIN);
		status = WAIT_FOR_MOTORS_TO_FINNISH; // set the state machine to wait for the motors to finish
		nextStatus = Running; // the next status to move to when the motors finish
		startTime = now + 1000; // the motors should be done in 1000 ms
		nextTime = startTime + 1000; // the next timer loop should be 1000ms after the motors stop
		break;
	case Running:
		ace.loop();
		// Set up a non-blocking 1000 ms delay
		status = WAIT_FOR_TIME;
		nextTime = nextTime + 100; // ensure no timer drift by incremeting the target
		// After 1000 ms, come back to this state
		nextStatus = Running;

		// Do something
		if (!digitalRead(BOOT_FLAG_PIN)) {
			Serial.println(
					" Running State Machine " + String((now - startTime)));
#if defined(USE_IMU)
			IMU->print();
#endif
#if defined(USE_IR_CAM)
			IRCamera->print();
#endif

			status = Scanning;
			//nextStatus = Scanning;
			scanningStatus = Driving;

		}
		break;

		/*	 case UltrasonicTest:
		Serial.println(Ultrasonic1.PingUltrasonic());

		PSEUDOCODE FOR PINGING ULTRASONIC AND DETERMINING LOCATION OF A BUILDING
	  bool areWeOnBlock1 = motor1->getAngleDegrees() > ace.mmTOdeg(380)  && motor1 ->getAngleDegrees() <  ace.mmTOdeg(430); //has motor 1 travelled at least 38cm and not passed the block at 43 cm
		bool areWeDrivingStraight = IMU->getEULER_azimuth() + ace.offset == 0;
		if(areWeOnBlock1 && areWeDrivingStraight && Ultrasonic1.PingUltrasonic() > 300 && Ultrasonic1.PingUltrasonic() < 400) {
			MapArray[2][6] = 1;
		}

		break;  */

	case UltrasonicTest:
		ping123 = Ultrasonic1.PingUltrasonic();
		if(ping123 > 300.0 && ping123 < 400.0) {
			buildingDistanceFromRobot = 1;
			Serial.println("X Coordinate: " + String(blocksTravelledX) + " Y Coordinate: " + String(buildingDistanceFromRobot));
			Serial.println(String(ping123));



		}

		if(ping123 > 1100.0 && ping123 < 1250.0 ) {
			buildingDistanceFromRobot = 3;
			Serial.println("X Coordinate: " + String(blocksTravelledX) + " Y Coordinate: 3");
			Serial.println(String(ping123));

		}

		if(ping123 > 1900.0 && ping123 < 2100.0) {
			buildingDistanceFromRobot = 5;
			Serial.println("X Coordinate: " + String(blocksTravelledX) + " Y Coordinate: 5");
			Serial.println(String(ping123));

		}
		Serial.println(String(ping123));
		status = UltrasonicTest;
		break;



	case WAIT_FOR_TIME:

		// Check to see if enough time has elapsed
		if (nextTime <= millis()) {
			// if the time is up, move on to the next state
			status = nextStatus;
		}
		break;
	case WAIT_FOR_MOTORS_TO_FINNISH:
		if (motor1->isInterpolationDone() && motor2->isInterpolationDone()
				&& motor3->isInterpolationDone()) {
			status = nextStatus;
		}
		break;
	case Halting:
		// save state and enter safe mode
		ace.driveStraight(0, 0, 50);
		Serial.println("Halting State machine");
		digitalWrite(H_BRIDGE_ENABLE, 0);
		motor3->stop();
		motor2->stop();
		motor1->stop();

		status = Halt;
		break;



	case Scanning:

		switch(scanningStatus){
		case Driving:
			if(blocksTravelledX == 0 && !previousFoundBuilding) { //&& checkForBuilding == false
				scanningStatus = ScanningBuilding;
				nextTime = millis() + 3500; //wait 3.5 seconds in the ScanninG Building state where ultrasonic will ping continously
			}
			if (!travelledXDistance) {  //have we completed driving 5 blocks for the x distance? && checkForBuilding == false
				Serial.println(String(blocksTravelledX));
				if(blocksTravelledX < 5) { //while we havent driven 5 blocks, drive one block at a time, and increment one each time
					Serial.println(blocksTravelledX);
					//ace.loop();

					if(trigger){
						target = blockDistance;
						target = ace.mmTOdeg(target) + (motor1->getAngleDegrees());
						trigger = false;
					}

					distanceError =  abs(this->motor1->getAngleDegrees()) - target;
					effort = 0.25 * distanceError;
					ace.driveStraight(-effort, 0, 200);

					if(motor1->getAngleDegrees() >= target){
						trigger = true;
						blocksTravelledX++;
						previousFoundBuilding = false;
						if(blocksTravelledX % 2 == 0 && !previousFoundBuilding) {
							scanningStatus = ScanningBuilding;
							nextTime = millis() + 2000; //wait 2 seconds in the ScanninG Building state where ultrasonic will ping continously

						}

					}


				}
				else if (blocksTravelledX == 5) {  //if we have travelled 5 blocks in the x direction, set x to true and y to false
					travelledXDistance = true;
					travelledYDistance = false;
				}
			}
			if(travelledXDistance == true && travelledYDistance == false && completedTurn == false ) {  //turn 90 degrees **HAS PROBLEMS GETS STUCK IN TURNDRIVE
				if(ace.turnDrive(0,90,10)) { //turnDrive has to be handled in its own separate "loop" or if statement
					scanningStatus = ScanningBuilding;
					nextTime = millis() + 2000;
					completedTurn = true;
				}


			}
			if (!travelledYDistance && completedTurn == true) {  //Repeat using Y direction
				Serial.println(String(blocksTravelledY));
				if(blocksTravelledY < 5) {
					Serial.println(blocksTravelledY);
					//ace.loop();
					if(trigger){
						target = blockDistance;
						target = ace.mmTOdeg(target) + (motor1->getAngleDegrees());
						trigger = false;
					}
					distanceError =  abs(this->motor1->getAngleDegrees()) - target;
					effort = 0.25 * distanceError;
					ace.driveStraight(-effort, 90, 200);
					if(motor1->getAngleDegrees() >= target){
						trigger = true;
						blocksTravelledY++;
						previousFoundBuilding = false;
						if(!(blocksTravelledY % 2 == 0) && !previousFoundBuilding) {
							scanningStatus = ScanningBuilding;
							nextTime = millis() + 2000; //wait 2 seconds in the ScanninG Building state where ultrasonic will ping continously
						}

					}
				}
				else if (blocksTravelledY == 5) {
					status = Halting;
				}
			}

			/*		if(needToTurn90) {
				if(ace.turnDrive(0,90,10)) {
					status = Pos3_4;
				}
			}
			else if (!finishedYCoordinate) {
				if(trigger){
					target = 1 * blockDistance;
					target = ace.mmTOdeg(target) + (motor1->getAngleDegrees());
					trigger = false;
				}
				distanceError =  abs(this->motor1->getAngleDegrees()) - target;
				effort = 0.25 * distanceError;
				ace.driveStraight(-effort, 0, 200);
				if(motor1->getAngleDegrees() >= target){
					status = ScanningBuilding;
					trigger = true;
					finishedYCoordinate = true;
				}
			} */




			break;

		case ScanningBuilding:
			motor1->setVelocityDegreesPerSecond(0);
			motor2->setVelocityDegreesPerSecond(0);
			ping123 = Ultrasonic1.PingUltrasonic();
			//had problems with going straight into the foundBuilding state and always reading building distance of 1
			if(ping123 > 300.0 && ping123 < 400.0  && millis() <= nextTime) {
				buildingDistanceFromRobot = 1;
				scanningStatus = foundBuilding;
			}

			if(ping123 > 1100.0 && ping123 < 1250.0  && millis() <= nextTime) {
				buildingDistanceFromRobot = 3;
				scanningStatus = foundBuilding;
			}

			if(ping123 > 1900.0 && ping123 < 2100.0  && millis() <= nextTime) {
				buildingDistanceFromRobot = 5;
				scanningStatus = foundBuilding;
			}

			else if (millis() >= nextTime) {
				scanningStatus = Driving;
			}
			break;

		case foundBuilding:
			if(!previousFoundBuilding && millis() <= nextTime) { //event checking making sure building only gets checked one time
				if (blocksTravelledX < 5) {
					Serial.println("X Coordinate: " + String(blocksTravelledX) + " Y Coordinate: " + String(buildingDistanceFromRobot));
					buildingArray[blocksTravelledX][buildingDistanceFromRobot] = 1;
				}
				else if (blocksTravelledY < 5) {
					Serial.println("X Coordinate: " + String(buildingDistanceFromRobot) + " Y Coordinate: " + String(blocksTravelledY));
					buildingArray[buildingDistanceFromRobot][blocksTravelledY] = 1;
				}
				Serial.println(String(Ultrasonic1.PingUltrasonic()));
				previousFoundBuilding = true; //sets back to true to ensure this if statement only happens once per foundBuilding loop
			}
			if (millis() >= nextTime){
				Serial.println("Missed building");
				scanningStatus = Driving;
			}
			break;
		}
		break;

		case Searching:
			switch(searchingStatus) {

			case DriveToBuilding:
				break;

			case SearchAroundBuilding:
				break;

			}
			break;

			case Communication:

				break;





















				/*	case WAIT_FOR_DISTANCE:
		Serial.println("test");
		if(ace.turnDrive(200,45,25)){
			status = nextStatus;
		}
		break;
	case Pos1_2:

		if(trigger){
			target = 550;
			target = ace.mmTOdeg(target) + (motor1->getAngleDegrees());
			trigger = false;
		}

		distanceError =  abs(this->motor1->getAngleDegrees()) - target;
		effort = 0.25 * distanceError;

		if(goingForwards){
			ace.driveStraight(-effort, 0, 200);
		}else{
			ace.driveStraight(-effort, -180, 200);
		}
		Serial.println(target);
		if(motor1->getAngleDegrees() >= target){
			if(goingForwards){
				status = Pos2_3;
			}else{
				status = Halting;
			}
			trigger = true;
		}
		break;
	case Pos2_3:
		ace.loop();
		//	ace.driveStraight(0, 90, 25);
		if(goingForwards){
			if(ace.turnDrive(0,90,10)) {
				status = Pos3_4;
			}
		}else{
			if(ace.turnDrive(0,-180,10)) {
				status = Pos1_2;
			}
		}
		break;
	case Pos3_4:

		if(trigger){
			target = 150;
			target = ace.mmTOdeg(target) + (motor1->getAngleDegrees());
			trigger = false;
		}

		distanceError =  abs(this->motor1->getAngleDegrees()) - target;
		effort = 0.25 * distanceError;
		if(goingForwards){
			ace.driveStraight(-effort, 90, 200);
		}else{
			ace.driveStraight(-effort, -90, 200);
		}
		Serial.println(target
		);
		if(motor1->getAngleDegrees() >= target){
			if(goingForwards){
				status = oneEighty;
			}else{
				status = Pos2_3;
			}
			trigger = true;
		}
		break;
	case oneEighty:

		if(ace.turnDrive(0,-89.99,10)) {
			status = Pos3_4;
			goingForwards = false;
		}
		trigger = true;
		break; */

			case Halt:
				// in safe mode
				break;

	}
	digitalWrite(WII_CONTROLLER_DETECT, 0);
}

/**
 * This is run fast and should return fast
 *
 * You call the PIDMotor's loop function. This will update the whole motor control system
 * This will read from the concoder and write to the motors and handle the hardware interface.
 */
void StudentsRobot::pidLoop() {
	motor1->loop();
	motor2->loop();
	motor3->loop();
}



