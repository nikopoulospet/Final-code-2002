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
static double sumUltrasonicReadings = 0;  //used for averaging ultrasonic readings b/c they are inconsistant
static int countUltrasonicReadings = 0;

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

			status = Scanning;
			scanningStatus = Driving;
		//	ace.robotPose.setRobotPosition(5, 0);
			SearchingRun = true;


		}
		break;


	case Testing:

		if(ace.turnTo(180)){
			status = Testting2;		}

		break;

	case Testting2:
		if(ace.turnTo(0)){
			status = Testing;
		}
		break;

	case UltrasonicTest:
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
		Serial.println("ACTUAL MAP ARRAY");
		fieldMap.printMap();
		status = Halt;
		break;


	case Scanning:
		switch(scanningStatus){
		case Driving:
			if(blocksTravelledX == 0 && !previousFoundBuilding) {  //edge case when we start the program, check for any buildings in row 0
				scanningStatus = UltrasonicCalc;
				nextTime = millis() + 3500; //wait 3.5 seconds in the ScanninG Building state where ultrasonic will ping continously
			}
			if (!travelledXDistance) {//have we completed driving 5 blocks for the x distance?
				if(blocksTravelledX < 5) { //while we havent driven 5 blocks, drive one block at a time, and increment each time
					//Serial.println(blocksTravelledX);
					if(trigger){  //trigger keeps a one time set of our target distance each time we need to travel a block
						target = blockDistance;
						target = ace.mmTOdeg(target) + (motor1->getAngleDegrees()); //adds on the degrees that we need to travel to our current position instead of resetting encoders
						trigger = false;
					}
					distanceError =  abs(this->motor1->getAngleDegrees()) - target; //calculate distance error between our current position and final position
					effort = 0.25 * distanceError;
					ace.driveStraight(-effort, 0, 200);
					if(motor1->getAngleDegrees() >= target){ //if we have surpassed the target, allow for another set of target distance, increment block
						trigger = true;
						blocksTravelledX++;
						previousFoundBuilding = false;
						if(blocksTravelledX % 2 == 0 && !previousFoundBuilding) { //if we have travelled an even number of blocks, check if there is a building in that row for 2 seconds
							scanningStatus = UltrasonicCalc;
							nextTime = millis() + 2000; //wait 2 seconds in the ScanninG Building state where ultrasonic will ping continously
						}
					}
				}
				else if (blocksTravelledX == 5) {  //if we have travelled 5 blocks in the x direction, set x to true and y to false so we no longer travel in the x direction but prepare to travel in y
					travelledXDistance = true;
					travelledYDistance = false;
				}
			}
			if(travelledXDistance == true && travelledYDistance == false && completedTurn == false ) {  //turn approximately 90 degrees only once
				if(ace.turnTo(90)) { //turnDrive has to be handled in its own separate "loop" or if statement
					completedTurn = true;
				}
			}
			if (!travelledYDistance && completedTurn == true) {  //Repeat using Y direction
				//Serial.println(String(blocksTravelledY));
				if(blocksTravelledY <= 5) {
					//	Serial.println(blocksTravelledY);
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
			Serial.println(String(ultrasonicPing));
			if (ultrasonicPing > maxUltrasonicReading) {
				maxUltrasonicReading = ultrasonicPing;
			}
			if (millis() <= nextTime && ultrasonicPing != -1.0) {  //while we still have time to ping, and the result of our ping is a reasonable number, add it to our sum and increment a counter
				sumUltrasonicReadings += ultrasonicPing;
				countUltrasonicReadings++;
				scanningStatus = UltrasonicCalc; //repeat status until millis has surpassed nextTime
			}
			else if (millis() >= nextTime) { //if we have surpassed nextTime
				averageUltrasonicReadings = sumUltrasonicReadings / countUltrasonicReadings; //calculate the average
				sumUltrasonicReadings = 0; //reset static variables back to 0 to avoid rollover
				countUltrasonicReadings = 0;
				Serial.println("AVERAGE: " + String(averageUltrasonicReadings));
				scanningStatus = ScanningBuilding; //go to scanning building state
			}
			break;

		case ScanningBuilding:
			if(averageUltrasonicReadings > 150 && averageUltrasonicReadings < 400) {//conditionals to check for the distance
				if(blocksTravelledX < 5) {
					buildingDistanceFromRobot = 1; //building is at Y=1
					scanningStatus = foundBuilding; //handle placement of building in a map
					maxUltrasonicReading = 0;
				}
				else if (blocksTravelledY <=5) { //we want to check the building when we have reached Y = 5 due to the construction of the field
					buildingDistanceFromRobot = 4; //distances change when we travel in the Y coordinate, X = 4
					scanningStatus = foundBuilding;
					maxUltrasonicReading = 0;
				}
				break;
			}
			if(averageUltrasonicReadings > 950 && averageUltrasonicReadings < 1200.0 && maxUltrasonicReading < 1350) {
				if(blocksTravelledX < 5) {
					buildingDistanceFromRobot = 3;
					scanningStatus = foundBuilding;
					maxUltrasonicReading = 0;

				}
				else if (blocksTravelledY <=5) {
					buildingDistanceFromRobot = 2; //distances change when we travel in the Y coordinate
					scanningStatus = foundBuilding;
					maxUltrasonicReading = 0;

				}
				break;
			}
			if((averageUltrasonicReadings < 150 || averageUltrasonicReadings > 1200 ) || (averageUltrasonicReadings > 400 && averageUltrasonicReadings < 950 && maxUltrasonicReading > 1000) || (averageUltrasonicReadings > 950 && averageUltrasonicReadings < 1200.0 && maxUltrasonicReading > 1350)) { //no building in a row
				Serial.println("NO BUILDING IN A ROW"); //if building is out of range/ ultrasonic reads average in between 950 & 1200 but max is greater to avoid placing an inaccurate building, or to avoid mistaking no building for a roadblock
				buildingDistanceFromRobot = 0;
				scanningStatus = foundBuilding;
				Serial.println(String(maxUltrasonicReading));
				maxUltrasonicReading = 0;
				break;
			}
			if(averageUltrasonicReadings > 400 && averageUltrasonicReadings < 900 && maxUltrasonicReading < 1200) { //roadblock - large side facing towards you reads between 400 and 900 with a maximum under 1200
				if(blocksTravelledX < 5) {
					Serial.println("ROADBLOCK FOUND! IN X");
					buildingDistanceFromRobot = 7;
					scanningStatus = foundBuilding;
					maxUltrasonicReading = 0;

				}
				else if (blocksTravelledY <=5) {
					Serial.println("ROADBLOCK FOUND! IN Y");
					buildingDistanceFromRobot = 9; //distances change when we travel in the Y coordinate
					scanningStatus = foundBuilding;
					maxUltrasonicReading = 0;

				}
				break;
			}
			/////////////////////////////////////////////////////////SENDS ROBOT INTO SEARCHING MACHINE
			else if (blocksTravelledY == 5) {
				status = Halting;
			}
			/////////////////////////////////////////////////////////
			break;

		case foundBuilding:
			if(!previousFoundBuilding) { //event checking making sure building only gets checked one time
				if(blocksTravelledX < 5) {
					if(buildingDistanceFromRobot == 1) { // if the building is at Y = 1 given ultrasonic data
						Serial.println("X Coordinate: " + String(blocksTravelledX) + " Y Coordinate: " + String(buildingDistanceFromRobot));
						//Plot& is a reference to the actual plot in the map array, and allows us to directly modify values of those plots
						Plot& buildingPlot = fieldMap.getPlot(5-blocksTravelledX, buildingDistanceFromRobot); //5-X,1  //sets up all plots in the row of x = 4 (0,0 being in the top left hand corner of the field)
						Plot& ambiguousPlot1 = fieldMap.getPlot(5-blocksTravelledX, buildingDistanceFromRobot+2); //5-X,3
						Plot& ambiguousPlot2 = fieldMap.getPlot(5-blocksTravelledX,buildingDistanceFromRobot+4);  //5-X,5
						buildingPlot.filledPlot = true; //if we see a building right in front of us, the buildings behind it may also be buildings
						ambiguousPlot1.filledPlot = true;
						ambiguousPlot2.filledPlot = true;
					}
					else if(buildingDistanceFromRobot == 3) {
						Serial.println("X Coordinate: " + String(blocksTravelledX) + " Y Coordinate: " + String(buildingDistanceFromRobot));
						Plot& buildingPlot = fieldMap.getPlot(5-blocksTravelledX, buildingDistanceFromRobot); // 5-X,3
						Plot& ambiguousPlot1 = fieldMap.getPlot(5-blocksTravelledX, buildingDistanceFromRobot+2); // 5-X,5
						Plot& ambiguousPlot2 = fieldMap.getPlot(5-blocksTravelledX, buildingDistanceFromRobot-2);  // 5-X,1
						buildingPlot.filledPlot = true; //if we see a building in the 3rd column of the field, then the building in front is not a building, but the one behind may be a building
						ambiguousPlot1.filledPlot = true;
						ambiguousPlot2.filledPlot = false;
					}
					else if (buildingDistanceFromRobot == 0){
						Serial.println("X Coordinate: " + String(blocksTravelledX) + " Y Coordinate: " + String(buildingDistanceFromRobot));
						ace.buildingArray[blocksTravelledX][buildingDistanceFromRobot] = 1;  //add building coordinate to our map
						Plot& ambiguousPlot1 = fieldMap.getPlot(5-blocksTravelledX, 5); //5-X,5
						ambiguousPlot1.filledPlot = true; //if we cannot sense the back row of the buildings, set the one in the back to a plausible building

					}
					else if (buildingDistanceFromRobot == 7) {
						Serial.println("X Coordinate: " + String(blocksTravelledX) + " Y Coordinate: " + String(buildingDistanceFromRobot));
						Plot& roadBlockPlot = fieldMap.getPlot(5-blocksTravelledX, 2); // 5-X,2
						Plot& ambiguousPlot1 = fieldMap.getPlot(5-blocksTravelledX, 3); // 5-X,3
						Plot& ambiguousPlot2 = fieldMap.getPlot(5-blocksTravelledX, 5);  // 5-X,5
						roadBlockPlot.filledPlot = true; //if we find a road block in 2nd column of the field, then set the buildings behind it to be possible buildings
						ambiguousPlot1.filledPlot = true;
						ambiguousPlot2.filledPlot = true;
					}
				}
				else if (blocksTravelledY <= 5) {
					if(buildingDistanceFromRobot == 4) {
						Serial.println("X Coordinate: " + String(blocksTravelledX) + " Y Coordinate: " + String(buildingDistanceFromRobot));
						Plot& buildingPlot = fieldMap.getPlot(5 - buildingDistanceFromRobot, blocksTravelledY); //1,Y
						Plot& ambiguousPlot1 = fieldMap.getPlot(5 - buildingDistanceFromRobot + 2, blocksTravelledY); //3,Y
						Plot& ambiguousPlot2 = fieldMap.getPlot(5 - buildingDistanceFromRobot + 4, blocksTravelledY); //5,Y
						buildingPlot.filledPlot = true; //ensure that we do not overwrite previous data, and in the y we only handle the spaces in front of the buildings we measure
					}
					else if(buildingDistanceFromRobot == 2) { //if building is in ROW 2
						Serial.println("X Coordinate: " + String(blocksTravelledX) + " Y Coordinate: " + String(buildingDistanceFromRobot));
						Plot& buildingPlot = fieldMap.getPlot(5 - buildingDistanceFromRobot, blocksTravelledY);  //3,Y
						Plot& ambiguousPlot1 = fieldMap.getPlot(5 - buildingDistanceFromRobot + 2, blocksTravelledY); //5,Y
						Plot& ambiguousPlot2 = fieldMap.getPlot(5 - buildingDistanceFromRobot - 2, blocksTravelledY); //1,Y
						buildingPlot.filledPlot = true;
						ambiguousPlot2.filledPlot = false; //set space in front of building to false
					}
					else if(buildingDistanceFromRobot == 0) {//if building is in ROW 2
						Serial.println("X Coordinate: " + String(blocksTravelledX) + " Y Coordinate: " + String(buildingDistanceFromRobot));
						Plot& buildingPlot = fieldMap.getPlot(5, blocksTravelledY);  //3,Y
						Plot& ambiguousPlot1 = fieldMap.getPlot(3, blocksTravelledY); //5,Y
						Plot& ambiguousPlot2 = fieldMap.getPlot(1, blocksTravelledY); //1,Y
						if(buildingPlot.filledPlot == false) {
							buildingPlot.filledPlot = false;
							ambiguousPlot1.filledPlot = false;
							ambiguousPlot2.filledPlot = false;
						}
						else {
							buildingPlot.filledPlot = true;
							ambiguousPlot1.filledPlot = false;
							ambiguousPlot2.filledPlot = false;
						}
					}
					else if(buildingDistanceFromRobot == 9) { //if building is in ROW 2
						Serial.println("X Coordinate: " + String(blocksTravelledX) + " Y Coordinate: " + String(buildingDistanceFromRobot));
						Plot& buildingPlot = fieldMap.getPlot(2, blocksTravelledY);  //2,Y
						Plot& ambiguousPlot1 = fieldMap.getPlot(3, blocksTravelledY); //3,Y
						Plot& ambiguousPlot2 = fieldMap.getPlot(5, blocksTravelledY); //5,Y
						Plot& finalizedOpen = fieldMap.getPlot(1, blocksTravelledY);
						buildingPlot.filledPlot = true;
						finalizedOpen.filledPlot = false;
					}
				}
				previousFoundBuilding = true; //sets back to true to ensure this if statement only happens once per foundBuilding loop
			}
			else if (blocksTravelledY == 5) {
				status = Halting;
			}
			else {
				scanningStatus = Driving;
			}
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



			case driveToRow:
				Serial.println("Drive to row");
				//if RB go to handleRB
				//next row reached go to search row (done)
				if(firstRun){
					//ace.robotPose.setRobotPosition(5, 0);
					TestingVar = 0;
					row += 2;
					firstRun = false;
				}

				if(row > 5){
					status = Halting; // return to home if no beacon detected
				}


				if(fieldMap.inRow(row)){
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
					searchingStatus = HandleRoadBlock;
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
					//buildingsSearched = TestingVar;
				}
				Serial.println(buildingToSearch);
				Serial.println(buildingsPerRow);
				//Serial.println(buildingsSearched);

//				if(buildingsSearched >= buildingsPerRow){
//					searchingStatus = driveToRow;
//				}
//
//				if(buildingsSearched < buildingsPerRow){
					if(buildingToSearch == 0){
						firstRun = true;
						searchingStatus = driveToRow; // no more buildings in row
					}else{
						Serial.println(row);
						if(ace.driveTo(buildingToSearch, row -1)){ // go to spot above building
								firstRun = true;
								searchingStatus = orient;
						}
					}
				//}else{firstRun = true;searchingStatus = driveToRow;}
				//check RB
				if(RoadBlockDetected){
					previousStatus = searchRow;
					firstRun = true;
					searchingStatus = HandleRoadBlock;
					currentTargetX = buildingToSearch;
					currentTargetY = row - 1;
					break;
				}

				break;

			case orient:
				Serial.println("ORIENTING+++++++++++++++++++++++++++++++++++++++++++++++++++");
				//oriented, go to look for robin
				if(firstRun){
					firstRun = false;
					orienting = false;
					orientation = ace.getOrientation(buildingToSearch, row); // int representing how to orient the robot will go wrong if robot is not directly NSE or W of the building
				}
				Serial.println(row);
				Serial.println(buildingToSearch);
				if(orientation == 1 && !orienting){
					orienting = true;
					orientHeading = 0;
				}
				if(orientation == 2 && !orienting){
					orienting = true;
					orientHeading = 90;
				}
				if(orientation == 3 && !orienting){
					orienting = true;
					orientHeading = 180;
				}
				if(orientation == 4 && !orienting){
					orienting = true;
					orientHeading = 270;
				}

				if(orienting && ace.turnTo(orientHeading)){
					firstRun = true;
					//status = Halting;
					searchingStatus = lookForRobin;
				}

				break;

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
						//TestingVar++;
						firstRun = true;
						searchingStatus = searchRow;
					}
				}
				break;

			case turnCorner:
				Serial.println("turnTHECOrnerrrrrrrrrrrrrrrrrrrr");
				//if RB go to handleRB
				if(ace.turnTheCorner(true)){
					searchingStatus = lookForRobin;
				}

				if(RoadBlockDetected){
					previousStatus = turnCorner;
					firstRun = true;
					searchingStatus = HandleRoadBlock;
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

				if(/* roadblock X Y match targetPos X Y*/ false){
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
