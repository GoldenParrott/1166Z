#include "init.h"

void blueRingside() {

}





void blueGoalside() {

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

	

	switch (autonnumber) {
	case -2:
		drawRedRing();
		IntakePTOPiston.set_value(true);
		break;
	case -1:
		drawRedMogo();
		IntakePTOPiston.set_value(false);
		break;
	case 1: 
		drawBlueMogo(); 
		IntakePTOPiston.set_value(false);
		break;
	case 2:
		drawBlueRing();
		IntakePTOPiston.set_value(true);
		break;
	}
	pros::delay(50);
}