/*
 * Map.cpp
 *
 *  Created on: Nov 23, 2019
 *      Author: Brian
 */

#include "Map.h"
#include "Plot.h"
//RowColumn
Map::Map() : zerozero(true,false,false,false), zeroone(true,false,false,false), zerotwo(true,false,false,false), zerothree(true,false,false,false), zerofour(true,false,false,false) , zerofive(true,false,false,false),
onezero(true,false,false,false), oneone(false, false, false, false, "200 Oak Street", "500 2nd Avenue", "100 Beech Street", "600 1st Avenue"), onetwo(true,false,false,false),
onethree(false, false, false, false, "400 Oak Street", "500 3rd Avenue", "300 Beech Street", "600 2nd Avenue"), onefour(true,false,false,false), onefive(false, false, false, false, "600 Oak Street", "No Address", "500 Beech Street", "600 3rd Avenue"),
twozero(true,false,false,false), twoone(true,false,false,false), twotwo(true,false,false,false), twothree(true,false,false,false), twofour(true,false,false,false), twofive(true,false,false,false),
threezero(true,false,false,false), threeone(false, false, false, false, "200 Beech Street", "300 2nd Avenue", "100 Maple Street", "400 1st Avenue"), threetwo(true,false,false,false),
threethree(false, false, false, false, "400 Beech Street", "300 3rd Avenue", "300 Maple Street", "400 2nd Avenue"), threefour(true,false,false,false), threefive(false, false, false, false, "600 Beech Street", "No Address", "500 Maple Street", "400 3rd Avenue"),
fourzero(true,false,false,false), fourone(true,false,false,false), fourtwo(true,false,false,false), fourthree(true,false,false,false), fourfour(true,false,false,false), fourfive(true,false,false,false),
fivezero(true,false,false,false), fiveone(false, false, false, false, "200 Maple Street", "100 2nd Avenue", "No Address", "200 1st Avenue"), fivetwo(true,false,false,false),
fivethree(false, false, false, false, "400 Maple Street", "100 3rd Avenue", "No Address", "200 2nd Avenue"), fivefour(true,false,false,false), fivefive(false, false, false, false, "600 Maple Street", "No Address", "No Address", "200 3rd Avenue")
{


};

void Map::printMap() {
	for (int i = 0; i < 6; i++) {
	        for (int j = 0; j < 6; j++) {
	            Serial.print(String(map[i][j].road));
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
	return 4; // do this later lmao
}






