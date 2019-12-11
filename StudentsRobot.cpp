/*
 * StudentsRobot.cpp
 *
 *  Created on: Dec 28, 2018
 *      Author: hephaestus
 *      Author: Peter Nikopoulos
 */

#include "StudentsRobot.h"
#define wheelTrackMM  225   //pass in wheeltrack and wheel radius into mm
#define	wheelRadiusMM 25.6
static double sumUltrasonicReadings = 0; //used for averaging ultrasonic readings b/c they are inconsistant
static int countUltrasonicReadings = 0;

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

boolean IRdetected = false;
int interruptCounter = 0;

void setPiezoStatus() {
	IRdetected = true;
	interruptCounter++;
	//Serial.println("INTERRUPT TRIGGED");

}

StudentsRobot::StudentsRobot(PIDMotor * motor1, PIDMotor * motor2,
		PIDMotor * motor3, Servo * servoTurret, Servo * servoLadder,
		IRCamSimplePacketComsServer * IRCam, GetIMU * imu) :ace(motor1, motor2, wheelTrackMM, wheelRadiusMM, imu), Ultrasonic1(), fieldMap(), Ultrasonic2()

{
	Serial.println("StudentsRobot::StudentsRobot constructor called here ");
	delay(500);
	this->servoTurret = servoTurret;
	this->servoLadder = servoLadder;
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
	motor2->velocityPID.setpid(0.004, 0.0007, 0);
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

	//SENSOR
	Ultrasonic1.attach(TrigPIN, EchoPIN);
	Ultrasonic2.attach(TrigPIN2, EchoPIN2);

	//SERVO
	pinMode(SERVO_TURRET_PIN, OUTPUT);
	pinMode(SERVO_LADDER_PIN, OUTPUT);

	//PIEZO
	ledcSetup(CHANNEL, FREQ, RESOLUTION);
	ledcAttachPin(PIEZO_PIN, CHANNEL);

	//IR INTERRUPT
	pinMode(36, INPUT);
	//attachInterrupt(36, &setPiezoStatus, LOW);

	//	publishAddress(0, 1, 1, 1);

}
/**
 * Seperate from running the motor control,
 * update the state machine for running the final project code here
 */

//SERVO HELPERS
void turretCenter(Servo x) {
	x.write(1640);
}
void turretLeft(Servo x) {
	x.write(2490);
}
void turretRight(Servo x) {
	x.write(790);
}
void ladderDeploy(Servo x) {
	x.write(2350);
}
void ladderDeployEdgeCase(Servo x) {
	x.write(1700);
}
void ladderHolster(Servo x) {
	x.write(740);
}

//PIEZO HELPERS
int checkNoteDuration() {
	if (longNote == true) {
		return longNoteDuration;
	} else if (extraLongNote == true) {
		return extraLongNoteDuration;
	} else
		return noteDuration;
	longNote = false;
}

int checkPauseDuration() {
	if (longPause == true) {
		return longPauseDuration;
	} else
		return pauseDuration;
}




void StudentsRobot::updateStateMachine() {
	digitalWrite(WII_CONTROLLER_DETECT, 1);
	long now = millis();
	if (status != StartupRobot){
		ace.loop();
		scanBeacon();
		/*	if(IRdetected && interruptCounter <= 1) { //if interrupt is triggered and beacon is detected
			status = piezzoBuzzer;
		} */
	}
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
			scanningStatus = Driving;
			searchingStatus = driveToRow;
			SearchingRun = true;
		}
		break;

	case Testing:
		ace.driveStraight(200, 0, 1000);
		break;

	case Testting2:
		if (ace.turnTo(0)) {
			status = Testing;
		}
		break;

	case UltrasonicTest:
		ultrasonicPing2 = Ultrasonic2.PingUltrasonic();
		Serial.println(String(ultrasonicPing2));
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
		switch (scanningStatus) {
		case Driving:
			turretRight(*servoTurret);
			ladderHolster(*servoLadder);
			if (blocksTravelledX == 0 && !previousFoundBuilding) { //edge case when we start the program, check for any buildings in row 0
				scanningStatus = UltrasonicCalc;
				nextTime = millis() + 3500; //wait 3.5 seconds in the ScanninG Building state where ultrasonic will ping continously
			}
			if (!travelledXDistance) { //have we completed driving 5 blocks for the x distance?
				if (blocksTravelledX < 5) { //while we havent driven 5 blocks, drive one block at a time, and increment each time
					//Serial.println(blocksTravelledX);
					if (ace.driveOneBlock()) {
						blocksTravelledX++;
						previousFoundBuilding = false;
						if (blocksTravelledX % 2 == 0
								&& !previousFoundBuilding) { //if we have travelled an even number of blocks, check if there is a building in that row for 2 seconds
							scanningStatus = UltrasonicCalc;
							nextTime = millis() + 2000; //wait 2 seconds in the ScanninG Building state where ultrasonic will ping continously
						}
					}
				} else if (blocksTravelledX == 5) { //if we have travelled 5 blocks in the x direction, set x to true and y to false so we no longer travel in the x direction but prepare to travel in y
					travelledXDistance = true;
					travelledYDistance = false;
				}
			}
			if (travelledXDistance == true && travelledYDistance == false
					&& completedTurn == false) { //turn approximately 90 degrees only once
				if (ace.turnTo(90)) { //turnDrive has to be handled in its own separate "loop" or if statement
					//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					completedTurn = true;
				}
			}
			if (!travelledYDistance && completedTurn == true) { //Repeat using Y direction
				if (blocksTravelledY < 5) { //while we havent driven 5 blocks, drive one block at a time, and increment each time
					//Serial.println(blocksTravelledX);
					if (ace.driveOneBlock()) {
						blocksTravelledY++;
						previousFoundBuilding = false;
						if (!(blocksTravelledY % 2 == 0)
								&& !previousFoundBuilding) { //if we have travelled an even number of blocks, check if there is a building in that row for 2 seconds
							scanningStatus = UltrasonicCalc;
							nextTime = millis() + 2000; //wait 2 seconds in the ScanninG Building state where ultrasonic will ping continously
						}
					}
				}
			}
			break;

		case UltrasonicCalc:
			motor1->setVelocityDegreesPerSecond(0); //halt the motors while we ping the buildings
			motor2->setVelocityDegreesPerSecond(0);
			ultrasonicPing = Ultrasonic1.PingUltrasonic();
			//	Serial.println(String(ultrasonicPing));
			if (ultrasonicPing > maxUltrasonicReading) {
				maxUltrasonicReading = ultrasonicPing;
			}
			if (millis() <= nextTime && ultrasonicPing != -1.0) { //while we still have time to ping, and the result of our ping is a reasonable number, add it to our sum and increment a counter
				sumUltrasonicReadings += ultrasonicPing;
				countUltrasonicReadings++;
				scanningStatus = UltrasonicCalc; //repeat status until millis has surpassed nextTime
			} else if (millis() >= nextTime) { //if we have surpassed nextTime
				averageUltrasonicReadings = sumUltrasonicReadings
						/ countUltrasonicReadings; //calculate the average
				sumUltrasonicReadings = 0; //reset static variables back to 0 to avoid rollover
				countUltrasonicReadings = 0;
				Serial.println("AVERAGE: " + String(averageUltrasonicReadings));
				scanningStatus = ScanningBuilding; //go to scanning building state
			}
			break;

		case ScanningBuilding:
			if (averageUltrasonicReadings > 150
					&& averageUltrasonicReadings < 400) { //conditionals to check for the distance
				if (blocksTravelledX < 5) {
					buildingDistanceFromRobot = 1; //building is at Y=1
					scanningStatus = foundBuilding; //handle placement of building in a map
					maxUltrasonicReading = 0;
				} else if (blocksTravelledY <= 5) { //we want to check the building when we have reached Y = 5 due to the construction of the field
					buildingDistanceFromRobot = 4; //distances change when we travel in the Y coordinate, X = 4
					scanningStatus = foundBuilding;
					maxUltrasonicReading = 0;
				}
				break;
			}
			if (averageUltrasonicReadings > 950
					&& averageUltrasonicReadings < 1200.0
					&& maxUltrasonicReading < 1350) {
				if (blocksTravelledX < 5) {
					buildingDistanceFromRobot = 3;
					scanningStatus = foundBuilding;
					maxUltrasonicReading = 0;

				} else if (blocksTravelledY <= 5) {
					buildingDistanceFromRobot = 2; //distances change when we travel in the Y coordinate
					scanningStatus = foundBuilding;
					maxUltrasonicReading = 0;

				}
				break;
			}
			if ((averageUltrasonicReadings < 150
					|| averageUltrasonicReadings > 1200)
					|| (averageUltrasonicReadings > 400
							&& averageUltrasonicReadings < 950
							&& maxUltrasonicReading > 1000)
							|| (averageUltrasonicReadings > 950
									&& averageUltrasonicReadings < 1200.0
									&& maxUltrasonicReading > 1350)) { //no building in a row
				Serial.println("NO BUILDING IN A ROW"); //if building is out of range/ ultrasonic reads average in between 950 & 1200 but max is greater to avoid placing an inaccurate building, or to avoid mistaking no building for a roadblock
				buildingDistanceFromRobot = 0;
				scanningStatus = foundBuilding;
				Serial.println(String(maxUltrasonicReading));
				maxUltrasonicReading = 0;
				break;
			}
			if (averageUltrasonicReadings > 400
					&& averageUltrasonicReadings < 900
					&& maxUltrasonicReading < 1200) { //roadblock - large side facing towards you reads between 400 and 900 with a maximum under 1200
				if (blocksTravelledX < 5) {
					Serial.println("ROADBLOCK FOUND! IN X");
					buildingDistanceFromRobot = 7;
					scanningStatus = foundBuilding;
					maxUltrasonicReading = 0;

				} else if (blocksTravelledY <= 5) {
					Serial.println("ROADBLOCK FOUND! IN Y");
					buildingDistanceFromRobot = 9; //distances change when we travel in the Y coordinate
					scanningStatus = foundBuilding;
					maxUltrasonicReading = 0;

				}
				break;
			}
			/////////////////////////////////////////////////////////SENDS ROBOT INTO SEARCHING MACHINE
			else if (blocksTravelledY == 5) {
				status = Searching;
				searchingStatus = driveToRow;
				ace.printTemporaryBuildingArray();
				Serial.println("ACTUAL MAP ARRAY");
				fieldMap.printMap();
				turretLeft(*servoTurret);
			}
			/////////////////////////////////////////////////////////
			break;

		case foundBuilding:
			if (!previousFoundBuilding) { //event checking making sure building only gets checked one time
				if (blocksTravelledX < 5) {
					if (buildingDistanceFromRobot == 1) { // if the building is at Y = 1 given ultrasonic data
						//	Serial.println("X Coordinate: " + String(blocksTravelledX) + " Y Coordinate: " + String(buildingDistanceFromRobot));
						//Plot& is a reference to the actual plot in the map array, and allows us to directly modify values of those plots
						Plot& buildingPlot = fieldMap.getPlot(
								buildingDistanceFromRobot,
								5 - blocksTravelledX); //5-X,1  //sets up all plots in the row of x = 4 (0,0 being in the top left hand corner of the field)
						Plot& ambiguousPlot1 = fieldMap.getPlot(
								buildingDistanceFromRobot + 2,
								5 - blocksTravelledX); //5-X,3
						Plot& ambiguousPlot2 = fieldMap.getPlot(
								buildingDistanceFromRobot + 4,
								5 - blocksTravelledX);  //5-X,5
						buildingPlot.filledPlot = true; //if we see a building right in front of us, the buildings behind it may also be buildings
						ambiguousPlot1.filledPlot = true;
						ambiguousPlot2.filledPlot = true;
					} else if (buildingDistanceFromRobot == 3) {
						//Serial.println("X Coordinate: " + String(blocksTravelledX) + " Y Coordinate: " + String(buildingDistanceFromRobot));
						Plot& buildingPlot = fieldMap.getPlot(
								buildingDistanceFromRobot,
								5 - blocksTravelledX); // 5-X,3
						Plot& ambiguousPlot1 = fieldMap.getPlot(
								buildingDistanceFromRobot + 2,
								5 - blocksTravelledX); // 5-X,5
						Plot& ambiguousPlot2 = fieldMap.getPlot(
								buildingDistanceFromRobot - 2,
								5 - blocksTravelledX);  // 5-X,1
						buildingPlot.filledPlot = true; //if we see a building in the 3rd column of the field, then the building in front is not a building, but the one behind may be a building
						ambiguousPlot1.filledPlot = true;
						ambiguousPlot2.filledPlot = false;
					} else if (buildingDistanceFromRobot == 0) {
						//Serial.println("X Coordinate: " + String(blocksTravelledX) + " Y Coordinate: " + String(buildingDistanceFromRobot));
						ace.buildingArray[blocksTravelledX][buildingDistanceFromRobot] =
								1;  //add building coordinate to our map
						Plot& ambiguousPlot1 = fieldMap.getPlot(5,
								5 - blocksTravelledX); //5-X,5
						ambiguousPlot1.filledPlot = true; //if we cannot sense the back row of the buildings, set the one in the back to a plausible building

					} else if (buildingDistanceFromRobot == 7) {
						//	Serial.println("X Coordinate: " + String(blocksTravelledX) + " Y Coordinate: " + String(buildingDistanceFromRobot));
						Plot& roadBlockPlot = fieldMap.getPlot(2,
								5 - blocksTravelledX); // 5-X,2
						Plot& ambiguousPlot1 = fieldMap.getPlot(3,
								5 - blocksTravelledX); // 5-X,3
						Plot& ambiguousPlot2 = fieldMap.getPlot(5,
								5 - blocksTravelledX);  // 5-X,5
						roadBlockPlot.filledPlot = true; //if we find a road block in 2nd column of the field, then set the buildings behind it to be possible buildings
						ambiguousPlot1.filledPlot = true;
						ambiguousPlot2.filledPlot = true;
					}
				} else if (blocksTravelledY <= 5) {
					if (buildingDistanceFromRobot == 4) {
						Serial.println(
								"X Coordinate: " + String(blocksTravelledX)
								+ " Y Coordinate: "
								+ String(buildingDistanceFromRobot));
						Plot& buildingPlot = fieldMap.getPlot(blocksTravelledY,
								5 - buildingDistanceFromRobot); //1,Y
						Plot& ambiguousPlot1 = fieldMap.getPlot(
								blocksTravelledY,
								5 - buildingDistanceFromRobot + 2); //3,Y
						Plot& ambiguousPlot2 = fieldMap.getPlot(
								blocksTravelledY,
								5 - buildingDistanceFromRobot + 4); //5,Y
						buildingPlot.filledPlot = true; //ensure that we do not overwrite previous data, and in the y we only handle the spaces in front of the buildings we measure
					} else if (buildingDistanceFromRobot == 2) { //if building is in ROW 2
						Serial.println(
								"X Coordinate: " + String(blocksTravelledX)
								+ " Y Coordinate: "
								+ String(buildingDistanceFromRobot));
						Plot& buildingPlot = fieldMap.getPlot(blocksTravelledY,
								5 - buildingDistanceFromRobot);  //3,Y
						Plot& ambiguousPlot1 = fieldMap.getPlot(
								blocksTravelledY,
								5 - buildingDistanceFromRobot + 2); //5,Y
						Plot& ambiguousPlot2 = fieldMap.getPlot(
								blocksTravelledY,
								5 - buildingDistanceFromRobot - 2); //1,Y
						buildingPlot.filledPlot = true;
						ambiguousPlot2.filledPlot = false; //set space in front of building to false
					} else if (buildingDistanceFromRobot == 0) { //if building is in ROW 2
						Serial.println(
								"X Coordinate: " + String(blocksTravelledX)
								+ " Y Coordinate: "
								+ String(buildingDistanceFromRobot));
						Plot& buildingPlot = fieldMap.getPlot(blocksTravelledY,
								5);  //3,Y
						Plot& ambiguousPlot1 = fieldMap.getPlot(
								blocksTravelledY, 3); //5,Y
						Plot& ambiguousPlot2 = fieldMap.getPlot(
								blocksTravelledY, 1); //1,Y
						if (buildingPlot.filledPlot == false) {
							buildingPlot.filledPlot = false;
							ambiguousPlot1.filledPlot = false;
							ambiguousPlot2.filledPlot = false;
						} else {
							buildingPlot.filledPlot = true;
							ambiguousPlot1.filledPlot = false;
							ambiguousPlot2.filledPlot = false;
						}
					} else if (buildingDistanceFromRobot == 9) { //if building is in ROW 2
						Serial.println(
								"X Coordinate: " + String(blocksTravelledX)
								+ " Y Coordinate: "
								+ String(buildingDistanceFromRobot));
						Plot& buildingPlot = fieldMap.getPlot(blocksTravelledY,
								2);  //2,Y
						Plot& ambiguousPlot1 = fieldMap.getPlot(
								blocksTravelledY, 3); //3,Y
						Plot& ambiguousPlot2 = fieldMap.getPlot(
								blocksTravelledY, 5); //5,Y
						Plot& finalizedOpen = fieldMap.getPlot(blocksTravelledY,
								1);
						buildingPlot.filledPlot = true;
						finalizedOpen.filledPlot = false;
					}
				}
				previousFoundBuilding = true; //sets back to true to ensure this if statement only happens once per foundBuilding loop
			} else if (blocksTravelledY == 5) {
				status = Searching;
				searchingStatus = driveToRow;
				ace.printTemporaryBuildingArray();
				Serial.println("ACTUAL MAP ARRAY");
				fieldMap.printMap();
				turretLeft(*servoTurret);

			} else {
				scanningStatus = Driving;
			}
			break;
		}
		break;

		//end of scanning SM//
		//begining of searching SM//
		case Searching: //buildings marked are 11 33 55
			switch (searchingStatus) {
			if (!SearchingRun) {
				ace.robotPose.setRobotPosition(5, 0);
				//fieldMap.oneone.filledPlot = true;
				SearchingRun = true;

				DeltaX = abs(ace.robotPose.posX) - abs(DeltaX);
				DeltaY = abs(ace.robotPose.posY) - abs(DeltaY);
			}
			if ((DeltaX != 0 || DeltaY != 0) && false) { //false is replaced by RB scan
				//trigger RoadBlock Scan every time robot moves a coordinate
				RoadBlockDetected = true;
			}

			if(!beaconSeen){
				beaconSeen = scanBeacon();
			}


			// = scanBeacon();

			case driveToRow:
				//		Serial.println("Drive to row");
				//if RB go to handleRB
				//next row reached go to search row (done)
				if (firstRun) {
					//ace.robotPose.setRobotPosition(5, 0);
					TestingVar = 0;
					row += 2;
					firstRun = false;
				}

				if (row > 5) {
					status = driveHome; // return to home if no beacon detected

				}

				if (fieldMap.inRow(row)) {
					if (ace.driveTo(2, row - 1)) {
						firstRun = true;
						searchingStatus = searchRow;

					}
				} else {
					row += 2;

				}
				//check RB
				if (RoadBlockDetected) {
					previousStatus = driveToRow;
					firstRun = true;
					searchingStatus = HandleRoadBlock;
					currentTargetX = 2;
					currentTargetY = row - 1;
					break;
				}
				break;

			case searchRow:
				//	Serial.print(
				//			"searchRow================================================");
				//if RB go to handleRB
				//building to search reached go to orient
				if (firstRun) {
					previousStatus = searchRow;
					firstRun = false;
					buildingsPerRow = fieldMap.buildingsPer(row);
					buildingToSearch = fieldMap.buildingToSearch(row);
					//buildingsSearched = TestingVar;
				}

				//	Serial.println(buildingToSearch);
				//	Serial.println(buildingsPerRow);
				//Serial.println(buildingsSearched);

				//				if(buildingsSearched >= buildingsPerRow){
				//					searchingStatus = driveToRow;
				//				}
				//
				//				if(buildingsSearched < buildingsPerRow){
				if (buildingToSearch == 0) {
					firstRun = true;
					searchingStatus = driveToRow; // no more buildings in row
				} else {
					//		Serial.println(row);
					if (ace.driveTo(buildingToSearch, row - 1)) { // go to spot above building
						firstRun = true;
						searchingStatus = orient;

					}
				}
				//}else{firstRun = true;searchingStatus = driveToRow;}
				//check RB
				if (RoadBlockDetected) {
					previousStatus = searchRow;
					firstRun = true;
					searchingStatus = HandleRoadBlock;
					currentTargetX = buildingToSearch;
					currentTargetY = row - 1;
					break;
				}

				break;

			case orient:
				//		Serial.println(
				//				"ORIENTING+++++++++++++++++++++++++++++++++++++++++++++++++++");
				//oriented, go to look for robin
				if (firstRun) {
					firstRun = false;
					orienting = false;
					orientation = ace.getOrientation(buildingToSearch, row); // int representing how to orient the robot will go wrong if robot is not directly NSE or W of the building
				}
				//		Serial.println(row);
				//		Serial.println(buildingToSearch);
				if (orientation == 1 && !orienting) {
					orienting = true;
					orientHeading = 0;
				}
				if (orientation == 2 && !orienting) {
					orienting = true;
					orientHeading = 90;
				}
				if (orientation == 3 && !orienting) {
					orienting = true;
					orientHeading = 180;
				}
				if (orientation == 4 && !orienting) {
					orienting = true;
					orientHeading = 270;
				}

				if (orienting && ace.turnTo(orientHeading)) {
					firstRun = true;
					//status = Halting;
					searchingStatus = lookForRobin;
				}

				break;

			case lookForRobin:
				//Serial.println("looking for Robin");
				//window empty go to turn corner
				//all windows checked go to driveToRow
				if (firstRun) {
					firstRun = false;
					windowsToSearch = fieldMap.windowsToSearch(buildingToSearch, row);
				}
				//Serial.println(windowsToSearch);

				if (scanBeacon()) {
					status = piezzoBuzzer;
					// ping window
					///////////////////////////////TOGGLE TURRET, PING IR, IF NO BUILDING SET WINDOWS TO SEARCH TO 0
					// Announcing state

				} else {
					if (windowsToSearch != 0) {
						if (scanBeacon()) {
							//status = piezzoBuzzer;
							Serial.println("BEACON DETECTED IN LOOKFORROBIN++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
							Serial.println("BEACON DETECTED IN LOOKFORROBIN++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
							Serial.println("BEACON DETECTED IN LOOKFORROBIN++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
							Serial.println("BEACON DETECTED IN LOOKFORROBIN++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
							//status = piezzoBuzzer;
							status = piezzoBuzzer;
							break;
						} else {
							windowsToSearch--;
							searchingStatus = turnCorner;
						}
					} else {
						if(ace.robotPose.posY == row){
							searchingStatus = returnToRow;
						} else{
							firstRun = true;
							searchingStatus = searchRow;
						}
					}
				}
					break;

			case turnCorner:
				//Serial.println("turnTHECOrnerrrrrrrrrrrrrrrrrrrr");
				//if RB go to handleRB
				if(fieldMap.edgeCase(buildingToSearch, row) == 1){ // edge case 1 is (1,5) corner building search CW , only one window
					switch (EC1) {
					turretRight(*servoTurret);
					case orientto2:
						if(ace.turnTo(90)){
							EC1 = turn;
						}
						break;
					case turn:
						turretRight(*servoTurret);
						if(ace.turnTheCorner(false)){
							//EC1 = orientto2;
							searchingStatus = lookForRobin;
						}
						break;
					}

				}else if(fieldMap.edgeCase(buildingToSearch, row) == 2){
					switch (EC2){ // broken========================================================================================
					case turnCCW:
						if(ace.turnTheCorner(true)){
							turretLeft(*servoTurret);
							Serial.println("turn the corner CCW for EC 2 +++++++++++++++++++++++++++++++++++");
							EC2 = orientto1;
							searchingStatus = lookForRobin;
						}
						break;
					case orientto1:
						Serial.println("turn to 0 for EC 2 +++++++++++++++++++++++++++++++++++");
						if(ace.turnTo(0)){
							EC2 = turnCW1;
							//searchingStatus = lookForRobin;
						}
						break;
					case turnCW1:
						Serial.println("turn the corner CW for EC 2 round 1+++++++++++++++++++++++++++++++++++");
						if(ace.turnTheCorner(false)){
							turretRight(*servoTurret);
							EC2 = turnCW2;
							searchingStatus = lookForRobin;
						}
						break;
					case turnCW2:
						Serial.println("turn the corner CW for EC 2  round 2+++++++++++++++++++++++++++++++++++");
						if(ace.turnTheCorner(false)){
							turretRight(*servoTurret);
							//EC2 = turnCCW;
							searchingStatus = lookForRobin;
						}
						break;
					}

				}else{
					turretLeft(*servoTurret);
					if(ace.turnTheCorner(true)){
						searchingStatus = lookForRobin;
					}

				}

				if(scanBeacon()) {
					//status = piezzoBuzzer;
					Serial.println("BEACON DETECTED IN TURN CORNER++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");

					status = piezzoBuzzer;
					break;
				}


				if (RoadBlockDetected) {
					previousStatus = turnCorner;
					firstRun = true;
					searchingStatus = HandleRoadBlock;
					break;
				}

				break;


			case returnToRow:
				Serial.println("returning to row ==========++++++++++++++============++++++++++++=========");
				if(ace.driveTo(ace.robotPose.posX, row - 1)){
					firstRun = true;
					searchingStatus = searchRow;
				}
				break;

			case HandleRoadBlock: // this is only 2.5 points. nuked for now
				break;
			}
			break;

				//end of searching SM//

			case Communication:


				if(((ace.robotPose.posX == 5) && (ace.getOrientation(buildingToSearch, row) == 3)) || ((ace.robotPose.posY == 5)  && ((ace.getOrientation(buildingToSearch, row) == 4) || (ace.getOrientation(buildingToSearch, row) == 2)))){
					Serial.println("OIUSADJDSADJSALKJDSALKJDSALKJLKJADLKJDSLKJAJKJLDSDSALKJ/////////////////////////////////////////////////////////////DS");
					ladderDeployEdgeCase(*servoLadder);
					if(millis() >= communicationTime) {
						ladderHolster(*servoLadder);
						ace.turnTo(ace.robotPose.returnRobotHeading(IMU->getEULER_azimuth() + 180));
						status = driveHome;
					}

				}
				else{
					//Serial.println("SOMETHING ACTUALLY THAT YOU CAN READ");
					if(ace.driveOneBlock()) {
						Serial.println("DSAALKAJDSLKAJD///////////////////////////////////////////////////////////OISALKDJSAD");
						//Serial.println("WOOOOOOOOOO");
						status = driveHome;
					}
					if(millis() >= ladderTime) {
						ladderDeploy(*servoLadder);
					}
					break;
				}
				break;

			case piezzoBuzzer:
				motor1->setVelocityDegreesPerSecond(0);
				motor2->setVelocityDegreesPerSecond(0);
				if (noteCount < 20) {

					unsigned long currentMillis = millis();

					if (outputTone) {
						if (currentMillis - previousMillis >= checkNoteDuration()) {
							previousMillis = currentMillis;
							//ledcWriteTone(CHANNEL, 0);
							ledcDetachPin(PIEZO_PIN);
							outputTone = false;
						}
					} else {
						if (currentMillis - previousMillis >= checkPauseDuration()) {
							previousMillis = currentMillis;
							ledcAttachPin(PIEZO_PIN, CHANNEL);
//
							if (noteCount == 1) {
								ledcWriteTone(CHANNEL, d);
							} else if (noteCount == 2) {
								ledcWriteTone(CHANNEL, d);
							} else if (noteCount == 3) {
								ledcWriteTone(CHANNEL, cSh);
							} else if (noteCount == 4) {
								ledcWriteTone(CHANNEL, cSh);
							} else if (noteCount == 5) {
								ledcWriteTone(CHANNEL, c);
							} else if (noteCount == 6) {
								ledcWriteTone(CHANNEL, c);
							} else if (noteCount == 7) {
								ledcWriteTone(CHANNEL, cSh);
							} else if (noteCount == 8) {
								ledcWriteTone(CHANNEL, cSh);
							} else if (noteCount == 9) {
								ledcWriteTone(CHANNEL, d);
							} else if (noteCount == 10) {
								ledcWriteTone(CHANNEL, d);
							} else if (noteCount == 11) {
								ledcWriteTone(CHANNEL, cSh);
							} else if (noteCount == 12) {
								ledcWriteTone(CHANNEL, cSh);
							} else if (noteCount == 13) {
								ledcWriteTone(CHANNEL, c);
							} else if (noteCount == 14) {
								ledcWriteTone(CHANNEL, c);
							} else if (noteCount == 15) {
								ledcWriteTone(CHANNEL, cSh);
							} else if (noteCount == 16) {
								ledcWriteTone(CHANNEL, cSh);
							} else if (noteCount == 17) {
								ledcWriteTone(CHANNEL, d);
								longNote = true;
								longPause = true;
							} else if (noteCount == 18) {
								ledcWriteTone(CHANNEL, d);
								longNote = false;
								longPause = false;
								extraLongNote = true;
							} else if (noteCount >= 19) {
								extraLongNote = false;
								ledcDetachPin(PIEZO_PIN);
							}

							noteCount = noteCount + 1;
							outputTone = true;
						}
					}
				} else {
					//IRdetected = 0;
					//Serial.println("HELOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO");
					publishAddress(ace.robotPose.posX, ace.robotPose.posY, buildingToSearch, row);
					//Serial.println("//////////////////////////////////////////////////////////////");
					Serial.println('Robot X:' + ace.robotPose.posX);
					Serial.println('Robot Y: '+ ace.robotPose.posY);
					Serial.println('Building X: ' + buildingToSearch);
					Serial.println('Building Y: '+ row);
					status = Communication;
					communicationTime = millis() + 30000;
					ladderTime = millis() + 1700;
				}
				break;



			case driveHome:

				if(firstRun){
					currentXpos = ace.robotPose.posX;
					currentYpos = ace.robotPose.posY;
					firstRun = false;
					if((ace.robotPose.posY == 0)||(ace.robotPose.posY == 2)||(ace.robotPose.posY == 4)){
						gH = toStart;
					}
				}
				//Serial.println("driving Home");
				switch(gH){
				case toNearest:
					//gH = toStart;
					Serial.println("toNearest");
					if(ace.driveTo(currentXpos, currentYpos - 1)){
						gH = toStart;
					}
					break;
				case toStart:
					Serial.println("toStart");
					if(ace.driveTo(0,5)){
						firstRun = true;
						status = Halting;
					}
					break;
				}
				break;
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

	//void IRAM_ATTR StudentsRobot::setStatusPiezo(){
	//	portENTER_CRITICAL_ISR();
	//	Serial.println("WE FOUND THE BEACON");
	//	portEXIT_CRITICAL_ISR();
	//}



	bool StudentsRobot::scanBeacon() {
		//read ADC, find amplitude
		float adc_val = analogRead(36);
		Serial.println(adc_val);

		//Inverting Schmitt trigger, detecting when drops low (150mV). High 1.75V when not detecting
		if (adc_val <= 1500) { //1.2V, since thresholds are far above and below there is no mistaking when it drops below
			Serial.println("Beacon detected!");
			return true;
		}

		else {
			return false;
		}
	}

	void StudentsRobot::publishAddress(int robot_x,
			int robot_y, int building_x, int building_y) {
		//	IMU->setXPosition(x_pos);
		//	IMU->setYPosition(y_pos);

		float msg;
		float msg1 = 100;
		float msg2 = 90;
		float msg3 = 9; //initially set to impossible values so we know if they are changed or not

		//Column 0
		if (robot_x == 0) {
			msg3 = 3;

			if (robot_y == 1) //600 1st Street: 103
				msg2 = 0;
			else if (robot_y == 3) //400 1st Street: 133
				msg2 = 30;
			else if (robot_y == 5) //200 1st Street: 163
				msg2 = 60;
		}

		//Row 0
		else if (robot_y == 0) {
			msg3 = 1;

			if (robot_x == 1) //200 Oak Street: 100
				msg2 = 0;
			else if (robot_x == 3) //400 Oak Street: 110
				msg2 = 10;
			else if (robot_x == 5) //600 Oak Street: 120
				msg2 = 20;
		}

		//Row 1
		else if (robot_y == 1) {
			if (robot_x == 2) {
				if (building_x == 1) { //500 2nd Street: 101
					msg2 = 0;
					msg3 = 1;
				} else if (building_x == 3) { //600 2nd Street: 113
					msg2 = 10;
					msg3 = 3;
				}
			} else if (robot_x == 4) {
				if (building_x == 3) { //500 3nd Street: 111
					msg2 = 10;
					msg3 = 1;
				} else if (building_x == 5) { //600 3nd Street: 123
					msg2 = 10;
					msg3 = 3;
				}
			}
		}

		//Row 2
		else if (robot_y == 2) {
			if (robot_x == 1) {
				if (building_y == 1) { //100 Beech Street: 102
					msg2 = 0;
					msg3 = 2;
				} else if (building_y == 3) { //200 Beech Street: 130
					msg2 = 30;
					msg3 = 0;
				}
			} else if (robot_x == 3) {
				if (building_y == 1) { //300 Beech Street: 112
					msg2 = 10;
					msg3 = 2;
				} else if (building_y == 3) { //400 Beech Street: 140
					msg2 = 40;
					msg3 = 0;
				}
			} else if (robot_x == 5) {
				if (building_y == 1) { //500 Beech Street: 122
					msg2 = 20;
					msg3 = 2;
				} else if (building_y == 3) { //600 Beech Street: 150
					msg2 = 50;
					msg3 = 0;
				}
			}
		}

		//Row 3
		else if (robot_y == 3) {
			if (robot_x == 2) {
				if (building_x == 1) { //300 2nd Street: 131
					msg2 = 30;
					msg3 = 1;
				} else if (building_x == 3) { //400 2nd Street: 143
					msg2 = 40;
					msg3 = 3;
				}
			} else if (robot_x == 4) {
				if (building_x == 3) { //300 3nd Street: 141
					msg2 = 40;
					msg3 = 1;
				} else if (building_x == 5) { //400 3nd Street: 153
					msg2 = 50;
					msg3 = 3;
				}
			}
		}

		//Row 4
		else if (robot_y == 4) {
			if (robot_x == 1) {
				if (building_y == 3) { //100 Maple Street: 132
					msg2 = 30;
					msg3 = 2;
				} else if (building_y == 5) { //200 Maple Street: 160
					msg2 = 60;
					msg3 = 0;
				}
			} else if (robot_x == 3) {
				if (building_y == 3) { //300 Maple Street: 142
					msg2 = 40;
					msg3 = 2;
				} else if (building_y == 5) { //400 Maple Street: 170
					msg2 = 70;
					msg3 = 0;
				}
			} else if (robot_x == 5) {
				if (building_y == 3) { //500 Maple Street: 152
					msg2 = 50;
					msg3 = 2;
				} else if (building_y == 5) { //600 Maple Street: 180
					msg2 = 80;
					msg3 = 0;
				}
			}
		}

		//Row 5
		else if (robot_y == 5) {
			if (robot_x == 2) {
				if (building_x == 1) { //100 2nd Street: 161
					msg2 = 60;
					msg3 = 1;
				} else if (building_x == 3) { //200 2nd Street: 173
					msg2 = 70;
					msg3 = 3;
				}
			} else if (robot_x == 4) {
				if (building_x == 3) { //100 3nd Street: 171
					msg2 = 70;
					msg3 = 1;
				} else if (building_x == 5) { //200 3nd Street: 183
					msg2 = 80;
					msg3 = 3;
				}
			}
		}

		msg = msg1 + msg2 + msg3;
		Serial.println(msg);
		IMU->setZPosition(msg);
	}


	//void StudentsRobot::getBuildingLocation(int robot_x, int robot_y, int heading) {
	//
	//}

