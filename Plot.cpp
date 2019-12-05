/*
 * Plot.cpp
 *
 *  Created on: Nov 23, 2019
 *      Author: Brian
 *      Author: Peter Nikopoulos
 */

#include "Plot.h"

Plot::Plot(bool road, bool filledPlot, bool searched, bool roadBlock, String addy1, String addy2, String addy3, String addy4, int Windows) {
	this->road = road;
	this->filledPlot = filledPlot;
	this->searched = searched;
	this->roadBlock = roadBlock;
	this->addy1 = addy1;
	this->addy2 = addy2;
	this->addy3 = addy3;
	this->addy4 = addy4;
	this->Windows = Windows;
}

Plot::Plot(bool road, bool filledPlot, bool searched, bool roadBlock, int Windows) {
	this->road = road;
	this->filledPlot = filledPlot;
	this->searched = searched;
	this->roadBlock = roadBlock;
	this->Windows = Windows;

}

Plot::~Plot() {
	// TODO Auto-generated destructor stub
}

/*
 * method for outputting proper address based on chosen plot and robot pos
 */

