#include <stdio.h>

#include "laneDetect.h"

int main(int argc, char** argv) {

	printf("This is from main\n");


	const char* img = "road.jpg";

	int k = getLaneStatus(imgPath);

	printf("got value of %d\n",k );
	return 0;
}