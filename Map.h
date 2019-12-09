/*
 * Map.h
 *
 *  Created on: Nov 23, 2019
 *      Author: Brian
 *      Author: Peter Nikopoulos
 */

#ifndef MAP_H_
#define MAP_H_
#include <math.h>
#include <Arduino.h>
#include "Plot.h"

class Map {
private:
	int buildingCounter = 0;
	int toSearch = 0;
public:
	Map();
	Plot zerozero;
	Plot zeroone;
	Plot zerotwo;
	Plot zerothree;
	Plot zerofour;
	Plot zerofive;
	Plot onezero;
	Plot oneone;
	Plot onetwo;
	Plot onethree;
	Plot onefour;
	Plot onefive;
	Plot twozero;
	Plot twoone;
	Plot twotwo;
	Plot twothree;
	Plot twofour;
	Plot twofive;
	Plot threezero;
	Plot threeone;
	Plot threetwo;
	Plot threethree;
	Plot threefour;
	Plot threefive;
	Plot fourzero;
	Plot fourone;
	Plot fourtwo;
	Plot fourthree;
	Plot fourfour;
	Plot fourfive;
	Plot fivezero;
	Plot fiveone;
	Plot fivetwo;
	Plot fivethree;
	Plot fivefour;
	Plot fivefive;
	Plot map[6][6] = {{zerozero,zeroone,zerotwo,zerothree,zerofour,zerofive},
			{onezero,oneone,onetwo,onethree,onefour,onefive},
			{twozero,twoone,twotwo,twothree,twofour,twofive},
			{threezero,threeone,threetwo,threethree,threefour,threefive},
			{fourzero,fourone,fourtwo,fourthree,fourfour,fourfive},
			{fivezero,fiveone,fivetwo,fivethree,fivefour,fivefive}};
	void printMap();

	bool inRow(int row);
	int buildingsPer(int row);
	int buildingToSearch(int row);
	int windowsToSearch(int x, int y);

	Plot& getPlot(int x, int y);

};




#endif /* MAP_H_ */
