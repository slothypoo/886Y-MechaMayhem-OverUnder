#include "main.h"

void leftArcade(){

   static bool driveToggle = false;

		double leftX = master.get_analog(ANALOG_LEFT_X);
		double leftY = master.get_analog(ANALOG_LEFT_Y);

		if(master.get_digital_new_press(DIGITAL_A) == 1){
			driveToggle = !driveToggle;
		}



		if (!driveToggle) { // Use !driveToggle instead of driveToggle == 0
        lF = leftY + leftX;
        lB = leftY + leftX;
        lM = leftY + leftX;
        rF = leftY + -leftX;
        rB = leftY + -leftX;
        rM = leftY + -leftX;
    	} 
		else {
        lF = -leftY + leftX;
        lB = -leftY + leftX;
        lM = -leftY + leftX;
        rF = -leftY + -leftX;
        rB = -leftY + -leftX;
        rM = -leftY + -leftX;
    	}

}


