#include <wiringPi.h>

#include "blink.h"

int toggleLight() {


	wiringPiSetup();
	pinMode(0, OUTPUT);
	for(;;) {
	   digitalWrite(0,HIGH); delay(500);
	   digitalWrite(0,LOW); delay(500);
	}
	return 1;
}
