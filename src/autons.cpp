#include "init.h"

void blueRingside() {

}


void blueGoalside() {
	//PIDTurner(findHeadingOfLine(universalCurrentLocation, {-24, -48}), 2);
	//PIDTurner(90, 2);
	PIDMover({-24, -48}, false);
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