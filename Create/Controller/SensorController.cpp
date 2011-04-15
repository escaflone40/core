/*
 * SensorController.cpp
 *
 *  Created on: Mar 15, 2011
 *      Author: abraham
 */

#include "SensorController.h"

#include "../COIL/ArduinoCOIL.h"
#include "../Library/Debug.h"
#include "../Library/SleeperThread.h"
#include "../MovementTracker/MovementTracker.h"


SensorController::SensorController(Create *create, int speed, int interval) :
	ArduinoController("Arduino", create, interval) {

	this->speed = speed;
	heading = 0;
	leftPinger = 0;
	rightPinger = 0;
	frontIR = 0;
	leftIR = 0;
	rightIR = 0;

	// Welcome VFF Algorithm
	vffAI = VFF();
	vffAI.run();
}

SensorController::~SensorController() {
}

void SensorController::run()
{
	// Enter processing loop
	stopRequested = false;
	while (stopRequested == false) {

//		if(create->arduino_active)
//		{
//			heading = arduino_getHeading();
//			frontIR = arduino_getFrontIR();
//			leftIR = arduino_getLeftIR();
//			rightIR = arduino_getRightIR();
//
//
//			Trafo2D point = create->movementTracker->transformation;
//
//			vffAI.run(rightIR, frontIR, leftIR, 0);
//			vffAI.run();
//		}
		long x = create->movementTracker->x();
		long y = create->movementTracker->y();
		printf("[SensorController] %d, %d\n", x/10, y/10);
		//point.print();
		//printf("%f,%f\n", point.trans().x(), point.trans().y());

		//vffAI.


		//Debug::print("[ArduinoController] Left: %1  front: %2  Right: %3  Heading: %4", leftIR, frontIR, rightIR, heading);
		//Debug::print("Heading: %1", heading);
		// Sleep our interval...
		this->msleep(interval);
	}
}

int SensorController::arduino_getHeading()
{
	return (int) create->arduino->readCompass();
}

int SensorController::arduino_getLeftPinger()
{
	return (int) create->arduino->readLeftPinger();
}

int SensorController::arduino_getRightPinger()
{
	return (int) create->arduino->readRightPinger();
}

int SensorController::arduino_getFrontIR()
{
	return (int) create->arduino->readInfraredFront();
}

int SensorController::arduino_getLeftIR()
{
	return (int) create->arduino->readInfraredLeft();
}

int SensorController::arduino_getRightIR()
{
	return (int) create->arduino->readInfraredRight();
}
