#include "init.h"

void blueRingside() {

}


void blueGoalside() {
/*
	auto print = []() {while (true) {
		Master.print(0, 0, "x = %f", universalCurrentLocation.x);
		Master.print(1, 0, "y = %f", universalCurrentLocation.y);
	}};
	pros::Task p(print);
*/

	PIDMover({48, 0});


	//PIDMover({48, -48}, false);
	//pros::delay(1000);
	//PIDMover({-48, -48}, true);

	pros::delay(500);
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