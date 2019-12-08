/*
 * Map.cpp
 *
 *  Created on: Nov 23, 2019
 *      Author: Brian
 *      Author: Peter Nikopoulos
 */

#include "Map.h"
#include "Plot.h"
//RowColumn
Map::Map() :
zerozero(true,false,false,false,0),
zeroone(true,false,false,false,0),
zerotwo(true,false,false,false,0),
zerothree(true,false,false,false,0),
zerofour(true,false,false,false,0),
zerofive(true,false,false,false,0),
onezero(true,false,false,false,0),
oneone(false, false, false, false, "200 Oak Street", "500 2nd Avenue", "100 Beech Street", "600 1st Avenue" , 4),
onetwo(true,false,false,false,0),
onethree(false, true, false, false, "400 Oak Street", "500 3rd Avenue", "300 Beech Street", "600 2nd Avenue" ,4),
onefour(true,false,false,false,0),
onefive(false, false, false, false, "600 Oak Street", "No Address", "500 Beech Street", "600 3rd Avenue" , 3),
twozero(true,false,false,false,0),
twoone(true,false,false,false,0),
twotwo(true,false,false,false,0),
twothree(true,false,false,false,0),
twofour(true,false,false,false,0),
twofive(true,false,false,false,0),
threezero(true,false,false,false,0),
threeone(false, false , false, false, "200 Beech Street", "300 2nd Avenue", "100 Maple Street", "400 1st Avenue" ,4),
threetwo(true,false,false,false,0),
threethree(false, false, false, false, "400 Beech Street", "300 3rd Avenue", "300 Maple Street", "400 2nd Avenue" ,4),
threefour(true,false,false,false,0),
threefive(false, false, false, false, "600 Beech Street", "No Address", "500 Maple Street", "400 3rd Avenue", 3),
fourzero(true,false,false,false,0),
fourone(true,false,false,false,0),
fourtwo(true,false,false,false,0),
fourthree(true,false,false,false,0),
fourfour(true,false,false,false,0),
fourfive(true,false,false,false,0),
fivezero(true,false,false,false,0),
fiveone(false, false, false, false, "200 Maple Street", "100 2nd Avenue", "No Address", "200 1st Avenue" ,3),
fivetwo(true,false,false,false,0),
fivethree(false, false, false, false, "400 Maple Street", "100 3rd Avenue", "No Address", "200 2nd Avenue",3),
fivefour(true,false,false,false,0),
fivefive(false, true, false, false, "600 Maple Street", "No Address", "No Address", "200 3rd Avenue" ,2)
{


};

void Map::printMap() {
	for (int i = 0; i < 6; i++) {
	        for (int j = 0; j < 6; j++) {
	            Serial.print(String(map[i][j].filledPlot));
	        }
	        printf("\n");
	    }
}

bool Map::inRow(int row){
	if(map[1][row].filledPlot){
		return true;
	}
	if(map[3][row].filledPlot){
		return true;
	}
	if(map[5][row].filledPlot){
		return true;
	}else{return false;}
}

int Map::buildingsPer(int row){
	for(int i = 1; i < 5; i+=2){
		if(map[i][row].filledPlot){
			buildingCounter++;
		}
	}
	return buildingCounter;
}

int Map::buildingToSearch(int row){
	if(map[1][row].filledPlot && !map[1][row].searched){
		map[1][row].searched = true;
		return 1;
	}
	if(map[3][row].filledPlot && !map[3][row].searched){
		map[3][row].searched = true;
		return 3;
	}
	if(map[5][row].filledPlot && !map[5][row].searched){
		map[5][row].searched = true;
		return 5;
	}
	else{return 0;}
}

int Map::windowsToSearch(int x, int y){
	return map[x][y].Windows;
}

Plot& Map::getPlot(int x, int y) {
	return map[x][y];
}







