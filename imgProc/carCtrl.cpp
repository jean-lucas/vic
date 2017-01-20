#include <stdio.h>


#include "carCtrl.h"
#include "laneDetect.h"

int main(int argc, char** argv) {

	printf("This is from main\n");


	int k = getLaneStatus();

	printf("got value of %d\n",k );
	return 0;
}