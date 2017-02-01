#include <stdio.h>


#include "carCtrl.h"
#include "laneDetect.h"
#include "carComms.h"



int main(int argc, char** argv) {

	printf("This is from main\n");


	int k = getLaneStatus();
	int i =	sendToIC("blah blah\n");
//	printf("got value of %d and %d\n",k, i );
	return 0;


}
