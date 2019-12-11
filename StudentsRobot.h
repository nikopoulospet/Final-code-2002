/*
 * StudentsRobot.h
 *
 *  Created on: Dec 28, 2018
 *      Author: hephaestus
 *      Author: Peter Nikopoulos
 */

#ifndef STUDENTSROBOT_H_
#define STUDENTSROBOT_H_
#include "config.h"
#include <Arduino.h>
#include "src/pid/ServoEncoderPIDMotor.h"
#include "src/pid/HBridgeEncoderPIDMotor.h"
#include "src/pid/ServoAnalogPIDMotor.h"
#include <ESP32Servo.h>

#include "Sensors.h"
#include "DrivingChassis.h"
#include "Map.h"
#include "src/commands/IRCamSimplePacketComsServer.h"
#include "src/commands/GetIMU.h"


/**
 * @enum RobotStateMachine
 * These are sample values for a sample state machine.
 * Feel free to add ot remove values from here
 */
enum RobotStateMachine {

	StartupRobot = 0, StartRunning = 1, Running = 2, Halting = 3, Halt = 4, WAIT_FOR_MOTORS_TO_FINNISH=5,WAIT_FOR_TIME=6, Searching = 14, Scanning = 15, Communication = 16, UltrasonicTest = 12, Testing = 13, Testting2 = 17,
	piezzoBuzzer = 19, driveHome = 20,
	//,WAIT_FOR_DISTANCE=7,Pos1_2 = 8,Pos2_3 = 9,Pos3_4 = 10, oneEighty = 11,UltrasonicTest = 12,

};

enum ScanningStateMachine {
	Driving = 0, ScanningBuilding = 1, foundBuilding = 2, UltrasonicCalc = 4,
};

enum SearchingStateMachine {
	driveToRow = 0, searchRow = 1, orient = 2, lookForRobin = 3, turnCorner = 4, HandleRoadBlock = 5, returnToRow = 6,
};

enum turningCorner {
	driveforwards = 0, turn90 = 1,
};

enum testing{
	test0 = 0, test1 = 1, test2 = 2,
};

enum edgeCase1{
	orientto2 = 0, turn = 1,
};

enum edgeCase2{
	turnCCW = 0, orientto1 = 1, turnCW1 = 2, turnCW2 = 3,
};
enum goHome{
	toNearest = 0,  toStart =1,
};
/**
 * @enum ComStackStatusState
 * These are values for the communications stack
 * Don't add any more or change these. This is how you tell the GUI
 * what state your robot is in.
 */
enum ComStackStatusState {
	Ready_for_new_task = 0,
	Heading_to_pickup = 1,
	Waiting_for_approval_to_pickup = 2,
	Picking_up = 3,
	Heading_to_Dropoff = 4,
	Waiting_for_approval_to_dropoff = 5,
	Dropping_off = 6,
	Heading_to_safe_zone = 7,
	Fault_failed_pickup = 8,
	Fault_failed_dropoff = 9,
	Fault_excessive_load = 10,
	Fault_obstructed_path = 11,
	Fault_E_Stop_pressed = 12
};
/**
 * @class StudentsRobot
 */
class StudentsRobot {
private:
	PIDMotor * motor1;
	PIDMotor * motor2;
	PIDMotor * motor3;
	Servo * servoTurret;
	Servo * servoLadder;
	float lsensorVal=0;
	float rsensorVal=0;
	long nextTime = 0;
    long startTime = 0;
    float targetDistPosition1To2 = -(55/ (2 * PI * 25.6)  * 360); //55cm divided by circumference of wheel times 360 degrees
    float targetDistPosition2To3 = -(15/15.96) * 360;
    float targetDistPosition3to4 = -(15/ (2 * PI * 25.6) * 360);
    //float targetDist = -1461.6;  //target distance for arc
    //float targetDist = -1352;  //target distance for driving straight
    DrivingChassis  ace;  //added driving chassis object for our robot
    Sensors Ultrasonic1;
    Map fieldMap;
    Sensors Ultrasonic2;
	RobotStateMachine nextStatus = StartupRobot;
	IRCamSimplePacketComsServer * IRCamera;
	GetIMU * IMU;

	//Searching Vars
	bool SearchingRun = false;
	int row = -1;
	bool firstRun = true;
	int buildingsSearched = 0;
	int buildingsPerRow = 0;
	int buildingToSearch = 0;
	int orientation = 0;
	bool oriented = false;
	int windowsToSearch = 0;
	bool firstDrive = true;

	int x = 2;
	int y = 1;
	double HX = 90;
	double HY = 0;

	int orientHeading = 0;
	bool orienting = false;

	int TestingVar = 0; // added because no way to update map yet!!!!!!!!!!!!!!!!!!!
	bool returnedToRow = false;

	//Halting state machine Vars
	SearchingStateMachine previousStatus = driveToRow;
	int DeltaX = 0;
	int DeltaY = 0;
	bool RoadBlockDetected = false;
	int currentXpos = 0;
	int currentYpos = 0;
	int currentTargetX = 0;
	int currentTargetY = 0;

	double Testheading = 0;

	bool run = true;

	edgeCase1 EC1 = orientto2;
	edgeCase2 EC2 = turnCCW;
	goHome gH = toNearest;

public:
	//volatile interupt for US sensor (add later)
	boolean trigger = true;
	double target = 0;
	double distanceError = 0;
	double effort = 0;
	boolean goingForwards = true;  //Lab 4 going forwards from position 1 to 2 is true
	double blockDistance = 405;  //mm distance of one block on the field
	int blocksTravelledX = 0;
	boolean travelledXDistance = false;
	boolean travelledYDistance = true;
	int blocksTravelledY = 0; //current position of robot in y coordinate
	boolean completedTurn = false; //have we completed the turn
	int buildingDistanceFromRobot = 0; //distance of building from Robot in blocks
	boolean previousFoundBuilding = false;
	double ultrasonicPing = 0; //Ultrasonic Reading Variable
	double averageUltrasonicReadings = 0;
	double maxUltrasonicReading = 0;
	boolean checkedForRoadBlock = false;
	double ultrasonicPing2 = 0;
	boolean fractionDistanceTrigger = true;
	double hardCodeDistance = 135;
	double target2 = 0;


	/**
	 * Constructor for StudentsRobot
	 *
	 * attach the 4 actuators
	 *
	 * these are the 4 actuators you need to use for this lab
	 * all 4 must be attached at this time
	 * DO NOT reuse pins or fail to attach any of the objects
	 *
	 */
	StudentsRobot(PIDMotor * motor1,
			PIDMotor * motor2, PIDMotor * motor3,
			Servo * servoTurret, Servo * servoLadder, IRCamSimplePacketComsServer * IRCam,GetIMU * imu);
	/**
	 * Command status
	 *
	 * this is sent upstream to the Java GUI to notify it of current state
	 */
	ComStackStatusState myCommandsStatus = Ready_for_new_task;
	/**
	 * This is internal data representing the runtime status of the robot for use in its state machine
	 */
	RobotStateMachine status = StartupRobot;
	ScanningStateMachine scanningStatus = Driving;
	SearchingStateMachine searchingStatus = driveToRow;
	turningCorner turningStep = driveforwards;
	testing testStep = test0;


	/**
	 * pidLoop This functoion is called to let the StudentsRobot controll the running of the PID loop functions
	 *
	 * The loop function on all motors needs to be run when this function is called and return fast
	 */
	void pidLoop();
	/**
	 * updateStateMachine use the stub state machine as a starting point.
	 *
	 * the students state machine can be updated with this function
	 */
	void updateStateMachine();

	bool scanBeacon();

	void publishAddress(float x_pos, float y_pos, int robot_x, int robot_y, int building_x, int building_y);

};

#endif /* STUDENTSROBOT_H_ */
