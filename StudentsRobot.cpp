/*
 * StudentsRobot.cpp
 *
 *  Created on: Dec 28, 2018
 *      Author: hephaestus
 */

#include "StudentsRobot.h"
#define wheelTrackMM  225   //pass in wheeltrack and wheel radius into mm
#define	wheelRadiusMM 25.6


//PIEZO
unsigned long previousMillis = 0;
const int pauseDuration = 100;
const int longPauseDuration = 200;
const int noteDuration = 100;
const int longNoteDuration = 200;
const int extraLongNoteDuration = 600;
boolean outputTone = false;
boolean longNote = false;
boolean extraLongNote = false;
boolean longPause = false;

int c = 523;
int cSh = 554;
int d = 587;
int noteCount = 1;


StudentsRobot::StudentsRobot(PIDMotor * motor1, PIDMotor * motor2,
		PIDMotor * motor3, Servo * servoTurret, Servo * servoLadder, IRCamSimplePacketComsServer * IRCam,
		GetIMU * imu) : ace(motor1,motor2,wheelTrackMM,wheelRadiusMM,imu), Ultrasonic1(), Ultrasonic2(), fieldMap()



{
	Serial.println("StudentsRobot::StudentsRobot constructor called here ");
	this->motor1 = motor1;
	this->motor2 = motor2;
	this->motor3 = motor3;
	this->servoTurret = servoTurret;
	this->servoLadder = servoLadder;
	IRCamera = IRCam;
	IMU = imu;
	ledcWriteTone(CHANNEL, d);
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
	// H-Bridge enable pin
	pinMode(H_BRIDGE_ENABLE, OUTPUT);
	// Stepper pins
	//pinMode(STEPPER_DIRECTION, OUTPUT);
	//pinMode(STEPPER_STEP, OUTPUT);
	// User button
	pinMode(BOOT_FLAG_PIN, INPUT_PULLUP);
	//Test IO
	pinMode(WII_CONTROLLER_DETECT, OUTPUT);

	//Servo
	pinMode(SERVO_TURRET_PIN, OUTPUT);
	pinMode(SERVO_LADDER_PIN, OUTPUT);
//	servoTurret->attach(SERVO_TURRET_PIN);
//	servoLadder->attach(SERVO_LADDER_PIN);
	//SENSOR
	Ultrasonic1.attach(Trig1PIN, Echo1PIN);
	Ultrasonic2.attach(Trig2PIN, Echo2PIN);
	ledcSetup(CHANNEL, FREQ, RESOLUTION);
	ledcAttachPin(PIEZO_PIN, CHANNEL);

}
/**
 * Seperate from running the motor control,
 * update the state machine for running the final project code here
 */

void turretCenter(Servo x){
	x.write(1640);
}

void turretLeft(Servo x){
	x.write(2490);
}

void turretRight(Servo x){
	x.write(790);
}

void ladderDeploy(Servo x){
	x.write(2350);
}

void ladderHolster(Servo x){
	x.write(740);
}



int checkNoteDuration(){
  if(longNote == true){return longNoteDuration;}
  else if(extraLongNote == true){return extraLongNoteDuration;}
  else return noteDuration;
  longNote = false;
}

int checkPauseDuration(){
  if(longPause == true){return longPauseDuration;}
  else return pauseDuration;
}



void StudentsRobot::updateStateMachine() {
	digitalWrite(WII_CONTROLLER_DETECT, 1);
	long now = millis();

	//polling for pose every 20ms, see DrivingChassis.cpp
	switch (status) {
	case StartupRobot:
		//Do this once at startup
		//status = UltrasonicTest;
		status = ServoTest;
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
		double blah;
		blah = Ultrasonic2.PingUltrasonic();
		if(blah != -1.0){
			Serial.println(blah);
		}
		break;

	case ServoTest:
		//servoTurret->write(1640);  //CW->decrease
		//Center=1640, 90CW=790, 90CCW=2490
		//servoLadder->write(740);  //CW->decrease
		//In Holster=740, Deployed=2350
		turretCenter(*servoTurret);
		ladderDeploy(*servoLadder);

		break;

	case PiezoTone:
		if(noteCount < 20){

		  unsigned long currentMillis = millis();

		  if (outputTone) {
		      if (currentMillis - previousMillis >= checkNoteDuration()) {
		          previousMillis = currentMillis;
		          //ledcWriteTone(CHANNEL, 0);
		          ledcDetachPin(PIEZO_PIN);
		          outputTone = false;
		          Serial.println("pause");
		      }
		  }
		  else {
		      if (currentMillis - previousMillis >= checkPauseDuration()) {
		          previousMillis = currentMillis;
		          ledcAttachPin(PIEZO_PIN, CHANNEL);

		          if(noteCount == 1){ledcWriteTone(CHANNEL, d);
		          Serial.println("d");}
		          else if(noteCount == 2){ledcWriteTone(CHANNEL, d);
		          Serial.println("d");}
		          else if(noteCount == 3){ledcWriteTone(CHANNEL, cSh);
		          Serial.println("cSh");}
		          else if(noteCount == 4){ledcWriteTone(CHANNEL, cSh);
		          Serial.println("cSh");}
		          else if(noteCount == 5){ledcWriteTone(CHANNEL, c);
		          Serial.println("c");}
		          else if(noteCount == 6){ledcWriteTone(CHANNEL, c);
		          Serial.println("c");}
		          else if(noteCount == 7){ledcWriteTone(CHANNEL, cSh);}
		          else if(noteCount == 8){ledcWriteTone(CHANNEL, cSh);
		          Serial.println("cSh");}
		          else if(noteCount == 9){ledcWriteTone(CHANNEL, d);}
		          else if(noteCount == 10){ledcWriteTone(CHANNEL, d);}
		          else if(noteCount == 11){ledcWriteTone(CHANNEL, cSh);}
		          else if(noteCount == 12){ledcWriteTone(CHANNEL, cSh);}
		          else if(noteCount == 13){ledcWriteTone(CHANNEL, c);}
		          else if(noteCount == 14){ledcWriteTone(CHANNEL, c);}
		          else if(noteCount == 15){ledcWriteTone(CHANNEL, cSh);}
		          else if(noteCount == 16){ledcWriteTone(CHANNEL, cSh);}
		          else if(noteCount == 17){ledcWriteTone(CHANNEL, d);
		            longNote = true;
		            longPause = true;
		          }
		          else if(noteCount == 18){ledcWriteTone(CHANNEL, d);
		            longNote = false;
		            longPause = false;
		            extraLongNote = true;
		          }
		          else if(noteCount>=19){
		            extraLongNote = false;
//			        digitalWrite(PIEZO_PIN, 0);
//		            ledcWriteTone(CHANNEL, 0);
		            ledcDetachPin(PIEZO_PIN);
			        Serial.println("finished");
		          }

		          noteCount = noteCount+1;
		          outputTone = true;
		      }
		  }
		}
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
		ace.printTemporaryBuildingArray();
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
				if(ace.turnDrive(0,88.5,10)) { //turnDrive has to be handled in its own separate "loop" or if statement
					completedTurn = true;
				}


			}
			if (!travelledYDistance && completedTurn == true) {  //Repeat using Y direction
				Serial.println(String(blocksTravelledY));
				if(blocksTravelledY <= 5) {
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
			if(ping123 > 200 && ping123 < 400.0  && millis() <= nextTime) {
				if(blocksTravelledX < 5) {
					buildingDistanceFromRobot = 1;
					scanningStatus = foundBuilding;
				}
				else if (blocksTravelledY <=5) {
					buildingDistanceFromRobot = 4; //distances change when we travel in the Y coordinate
					scanningStatus = foundBuilding;
				}
			}

			if(ping123 > 1000 && ping123 < 1200.0  && millis() <= nextTime) {
				if(blocksTravelledX < 5) {
					buildingDistanceFromRobot = 3;
					scanningStatus = foundBuilding;
				}
				else if (blocksTravelledY <=5) {
					buildingDistanceFromRobot = 2; //distances change when we travel in the Y coordinate
					scanningStatus = foundBuilding;
				}
			}

			/////////////////////////////////////////////////////////
			else if (blocksTravelledY == 5) {
				status = Halting;
			}
			/////////////////////////////////////////////////////////
			else if (millis() >= nextTime) {
				scanningStatus = Driving;
			}
			break;

		case foundBuilding:
			if(!previousFoundBuilding && millis() <= nextTime) { //event checking making sure building only gets checked one time
				if (blocksTravelledX < 5) {
					Serial.println("X Coordinate: " + String(blocksTravelledX) + " Y Coordinate: " + String(buildingDistanceFromRobot));
					ace.buildingArray[blocksTravelledX][buildingDistanceFromRobot] = 1;
				}
				else if (blocksTravelledY < 5) {
					Serial.println("X Coordinate: " + String(buildingDistanceFromRobot) + " Y Coordinate: " + String(blocksTravelledY));
					ace.buildingArray[buildingDistanceFromRobot][blocksTravelledY] = 1;
				}
				Serial.println(String(Ultrasonic1.PingUltrasonic()));
				previousFoundBuilding = true; //sets back to true to ensure this if statement only happens once per foundBuilding loop
			}

			else if (blocksTravelledY == 5) {
				status = Halting;
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




