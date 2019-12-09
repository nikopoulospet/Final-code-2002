/*
 * Plot.h
 *
 *  Created on: Nov 23, 2019
 *      Author: Brian
 *      Author: Peter Nikopoulos
 */

#ifndef PLOT_H_
#define PLOT_H_
#include <Arduino.h>

class Plot {
public:
	Plot();
	Plot(bool road, bool filledPlot, bool searched, bool roadBlock, String addy1, String addy2, String addy3, String addy4, int Windows);
	Plot (bool road, bool filledPlot, bool searched, bool roadBlock, int Windows);
	bool road;
	bool filledPlot;
	bool searched;
	bool roadBlock;
	String addy1;
	String addy2;
	String addy3;
	String addy4;
	int Windows;

	virtual ~Plot();
};

#endif /* PLOT_H_ */
