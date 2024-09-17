#include "init.h"

void blueRingside() {

}


void blueGoalside() {
	// PIDMover({-120, -60}, false);
	while (true) {
		pros::lcd::print(0, "x = %f", universalCurrentLocation.x);
		pros::lcd::print(1, "y = %f", universalCurrentLocation.y);
		pros::delay(50);
	}
}


void redGoalside() {

}


void redRingside() {

}

void autonSwitcher(){

	if (autonnumber == -1){
		autonnumber = 1;
	}else if(autonnumber == 1){
		autonnumber = -2;
	}else if(autonnumber == -2){
		autonnumber = 2;
	}else if(autonnumber == 2){
		autonnumber = -1;
	}else{
		autonnumber = 1;
	}

	
/*
	switch (autonnumber) {
	case -2:
		drawRedRing();
		break;
	case -1:
		drawRedMogo();
		break;
	case 1: 
		drawBlueMogo();
		break;
	case 2:
		drawBlueRing();
		break;
	} */
	pros::delay(50);
}