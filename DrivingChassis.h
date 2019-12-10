/*
 * DrivingChassis.h
 *
 *  Created on: Jan 12, 2019
 *      Author: hephaestus
 *      Author: Peter Nikopoulos
 */

#ifndef DRIVINGCHASSIS_H_
#define DRIVINGCHASSIS_H_
#include "src/pid/PIDMotor.h"
#include "src/commands/GetIMU.h"
#include "config.h"
#include "Pose.h"
/**
 * DrivingChassis encapsulates a 2 wheel differential steered chassis that drives around
 *
 * The 0,0,0 center of the robot is on the ground, half way between the left and right wheel contact points.
 *
 * The +X axis is the positive direction of travel
 *
 * The +Y axis goes from the center of the robot to the left wheel
 *
 * The +Z axis goes from the center up through the robot towards the ceiling.
 *
 * This object should manage the setting of motor setpoints to enable driving
 */


enum DriveToSteps {
	driveX = 0, toHeading1 = 1, driveY = 2, toHeading2 = 3, done =4,
};

enum CorneringSteps {
	drive = 0, turning = 1,
};


class DrivingChassis {
private:
	PIDMotor * myleft;
	PIDMotor * myright;
	GetIMU * IMU;

	//DRIVE TO VARS
	DriveToSteps Step = toHeading1;
	CorneringSteps corner = drive;

	//DRIVE PLOTS VARS
	double kpDistance = 7;
	double plotLen = 1200;
	double distOfset = 0;
	double targetDistance = 0;
	int changeInXpos = 0;
	int changeInYpos = 0;

	//turn corner Vars
	int driveCount = 0;
	int x;
	int y;
	bool decideDir = true;

	double startingHeading = 0;
	double dir = 0;

public:
	Pose robotPose;  //instantiating a pose object
	boolean loopFlag = false;
	long now = 0;
	double offset = 0;
	bool trigger = true;
	double wheelRadius = 25.6; // mm
	double distanceError = 0;
	double IMUheading = 0;
	double angleLeftMotor = 0;
	double angleRightMotor = 0;

	int Xdist = 0;
	int Ydist = 0;
	int heading1 = 0;
	int heading2 = 0;
	//double targetHeading = 0; //45 * (PI/180);  //hard coded heading in radians


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
	float distanceToWheelAngle(float distance);
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
	float chassisRotationToWheelDistance(float angle);
public:
	virtual ~DrivingChassis();

	/**
	 * DrivingChassis encapsulates a 2 wheel differential steered chassis that drives around
	 *
	 * @param left the left motor
	 * @param right the right motor
	 * @param wheelTrackMM is the measurment in milimeters of the distance from the left wheel contact point to the right wheels contact point
	 * @param wheelRadiusMM is the measurment in milimeters of the radius of the wheels
	 * @param imu The object that is used to access the IMU data
	 */
	DrivingChassis(PIDMotor * left, PIDMotor * right, float wheelTrackMM,
			float wheelRadiusMM,GetIMU * imu);

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
	void driveForward(float mmDistanceFromCurrent, int msDuration);
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
	void turnDegrees(float degreesToRotateBase, int msDuration);
	/**
	 * Check to see if the chassis is performing an action
	 *
	 * @return false is the chassis is driving, true is the chassis msDuration has elapsed
	 *
	 * @note this function is fast-return and should not block
	 */
	bool isChassisDoneDriving();
	/**
	 * loop()
	 *
	 * a fast loop function that will update states of the motors based on the information from the
	 * imu.
	 *
	 * @note this function is fast-return and should not block
	 */
	void loop();


	/**
	 * this function updates the pose of our robot as it drives
	 */

	void updatePose();

	void driveStraight(double speed, double targetHeading, int Kp);
	/**
	 * drives in the speed and direction specified, for driving straight a Kp of 100 is recommended, for turning a Kp of 25 is recommended
	 *
	 * @param speed is the target speed
	 * @param targetHeading is the desired heading of the robot
	 * @param Kp is the Kp for the controller used in the function
	 *
	 */

	//bool distanceDrive(double mm);
	/**
	 * drives to a distance in mm, using a p controller to handle speeds
	 *
	 * @param takes the desired drive distance in mm
	 *
	 * @return true when distance is reached
	 */

	//bool turnDriveEffortBool(double deg);
	/**
	 * turn function, turns to desired heading using driveStraight with zero speed and a modified heading
	 *
	 * @param takes the desired heading, positive is clockwise
	 *
	 * @return true when done turning, false until heading is reached
	 */

	double mmTOdeg(double mm);
	/**
	 * short conversion function for mm to degrees
	 *
	 * @param takes in the desired mm to convert
	 */

	void turn(double deg, double Kp);

	//void DrivingChassis::driveToCoordinate (int coord);
	/**
	 * Drives to some coordinate where each coord corresponds to a multiple of 40.5 cm distance
	 */

	bool turn90CCW();

	bool turn90CW();

	bool driveTo(int Xcord, int Ycord);

	int getOrientation(int building, int row);

	//bool drivePlotsinDir(double plots, double heading, bool isX);

	//bool turnDrive(double deg);

	bool turnTo(double deg);

	bool turnTheCorner(bool CCW);

	bool driveOneBlock();

	bool turnNum(int dir);

};





#endif /* DRIVINGCHASSIS_H_ */
