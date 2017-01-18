#include <stdio.h>

#include "laneDetect.h"

int main(int argc, char** argv) {

	printf("This is from main\n");


	const char* img = "offCenter.jpg";

	int k = getLaneStatus(img);

	printf("got value of %d\n",k );
	return 0;
}
