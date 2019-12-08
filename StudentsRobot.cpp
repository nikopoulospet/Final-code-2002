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
	ace.loop();
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

			status = Searching;
			//nextStatus = Scanning;
			scanningStatus = Driving;
			ace.robotPose.setRobotPosition(5, 0);
			SearchingRun = true;


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

	case Testing:
		if(ace.turn90CCW()){
			status = Halting;
		}
		break;

	case UltrasonicTest:
		fieldMap.printMap();
		status = Halting;
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
			if (!travelledXDistance) {  //have we completed driving 5 blocks for the x distance?
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
						scanningStatus = ScanningBuilding; //wait 5 seconds in the ScanninG Building state where ultrasonic will ping continously
						nextTime = millis() + 5000;

					}
				}
				else if (blocksTravelledX == 5) {  //if we have travelled 5 blocks in the x direction, set x to true and y to false
					travelledXDistance = true;
					travelledYDistance = false;
				}
			}
//			if(travelledXDistance == true && travelledYDistance == false && ace.turnDrive(90) && completedTurn == true) {  //turn 90 degrees **HAS PROBLEMS GETS STUCK IN TURNDRIVE
//				completedTurn = false;
//				scanningStatus = ScanningBuilding;
//				nextTime = millis() + 2000;
//
//			}
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
						scanningStatus = ScanningBuilding;
						nextTime = millis() + 2000;

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


			if(Ultrasonic1.PingUltrasonic() > 300 && Ultrasonic1.PingUltrasonic() < 400 && millis() <= nextTime) {
				Serial.println("HEADED HOME MOTHERFUCKERS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

			}
			else if (millis() >= nextTime) {
				scanningStatus = Driving;
			}



			break;

		case foundBuilding:

			break;



		}
		break;





		//end of scanning SM//
		//begining of searching SM//
		case Searching: //buildings marked are 11 33 55
			switch(searchingStatus) {
			if(!SearchingRun){
				ace.robotPose.setRobotPosition(5, 0);
				//fieldMap.oneone.filledPlot = true;
				SearchingRun = true;

				DeltaX = abs(ace.robotPose.posX) - abs(DeltaX);
				DeltaY = abs(ace.robotPose.posY) - abs(DeltaY);
			}
			if((DeltaX != 0 || DeltaY != 0) && false){ //false is replaced by RB scan
				//trigger RoadBlock Scan every time robot moves a coordinate
				RoadBlockDetected = true;
			}

			case driveToRow: // tested
				Serial.println("Drive to row");
				//if RB go to handleRB
				//next row reached go to search row (done)
				if(firstRun){
					//ace.robotPose.setRobotPosition(5, 0);
					TestingVar = 0;
					row = 1;
					firstRun = false;
				}

				if(row > 5){
					status = Halting;
				}

				//Serial.println(row);
				if(fieldMap.inRow(row)){
					//Serial.println("map" + String(fieldMap.inRow(row)));
					if(ace.driveTo(2, row - 1)){
						firstRun = true;
						searchingStatus = searchRow;
					}
				}else {
					row+=2;
				}
				//check RB
				if(RoadBlockDetected){
					previousStatus = driveToRow;
					firstRun = true;
					scanningStatus = HandleRoadBlock;
					currentTargetX = 2;
					currentTargetY = row - 1;
					break;
				}
				break;

			case searchRow:
				Serial.print("searchRow================================================");
				//if RB go to handleRB
				//building to search reached go to orient
				if(firstRun){
					previousStatus = searchRow;

					firstRun = false;
					buildingsPerRow = fieldMap.buildingsPer(row);
					buildingToSearch = fieldMap.buildingToSearch(row);
					buildingsSearched = TestingVar;
				}
				Serial.println(buildingToSearch);
				Serial.println(buildingsPerRow);
				Serial.println(buildingsSearched);

				if(row == 5 && buildingsSearched >= buildingsPerRow){
					status = Halting;
				}

				if(buildingsSearched < buildingsPerRow){
					if(buildingToSearch == 0){// this might not be doing anything...
						firstRun = true;
						searchingStatus = driveToRow; // no more buildings in row
					}else{
						Serial.println(row);
						if(ace.driveTo(buildingToSearch, row -1)){ // go to spot above building
							firstRun = true;
							searchingStatus = lookForRobin;
						}
					}
				}else{firstRun = true;searchingStatus = driveToRow;}
				//check RB
				if(RoadBlockDetected){
					previousStatus = searchRow;
					firstRun = true;
					scanningStatus = HandleRoadBlock;
					currentTargetX = buildingToSearch;
					currentTargetY = row - 1;
					break;
				}

				break;

//			case orient:
//				Serial.println("ORIENTING+++++++++++++++++++++++++++++++++++++++++++++++++++");
//				//oriented, go to look for robin
//				if(firstRun){
//					firstRun = false;
//					orienting = false;
//					orientation = ace.getOrientation(buildingToSearch, row); // int representing how to orient the robot will go wrong if robot is not directly NSE or W of the building
//				}
//				Serial.println(row);
//				Serial.println(buildingToSearch);
//				if(orientation == 1 && !orienting){
//					orienting = true;
//					orientHeading = 0;
//				}
//				if(orientation == 2 && !orienting){
//					orienting = true;
//					orientHeading = 90;
//				}
//				if(orientation == 3 && !orienting){
//					orienting = true;
//					orientHeading = 180;
//				}
//				if(orientation == 4 && !orienting){
//					orienting = true;
//					orientHeading = 270;
//				}
//
//				if(orienting && ace.turnTo(orientHeading)){
//					firstRun = true;
//					//status = Halting;
//					searchingStatus = lookForRobin;
//				}
//
//				break;

			case lookForRobin:
				Serial.println("looking for Robin");
				//window empty go to turn corner
				//all windows checked go to driveToRow
				if(firstRun){
					firstRun = false;
					windowsToSearch = fieldMap.windowsToSearch(buildingToSearch, row);
				}
				Serial.println(windowsToSearch);

				if(false){ // ping window
					// Announcing state
				} else{
					if(windowsToSearch != 0){
						windowsToSearch--;
						searchingStatus = turnCorner;
					}else{
						TestingVar++;
						firstRun = true;
						searchingStatus = searchRow;
					}
				}
				break;

			case turnCorner:
				Serial.println("turnTHECOrnerrrrrrrrrrrrrrrrrrrr");
				//if RB go to handleRB
				if(ace.turnTheCorner(true)){
					previousStatus = turnCorner;
					searchingStatus = lookForRobin;
					//status = ;
				}

				if(RoadBlockDetected){
					previousStatus = turnCorner;
					firstRun = true;
					scanningStatus = HandleRoadBlock;
					break;
				}

				break;

			case HandleRoadBlock:
				if(previousStatus == turnCorner){
					if(firstRun){
						firstRun = false;
						currentXpos = ace.robotPose.posX;
						currentYpos = ace.robotPose.posY;
						//Calculate roadblock X and Y

						//if robot is driving in Y pos
							//turn to 270
						//Else
							//turn to 0

						//widows to scan--;
					}

					// or add switch and set status to inPath

					//drive up 2 plots
					//2X CCW turn
					//drive up 2 plots.
				}

				if(/* roadblock X Y match targetPos X Y*/){
					if(previousStatus == driveToRow){
						//just drive around to another row pos position
						//back up to prev row
						//face 4
						//drive to x = 0 or next block
						//turn corner 2 X
						//end
					}else if(previousStatus == searchRow){
						//drive to next building side
						//either on 0 X side of RB -> CCW search
						//on 5 X side of RB -> CW search
						//mark window as read

					}
				}

				//handle RB when done return to previous state

				break;

			}
			break;
		//end of searching SM//

			case Communication:

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



