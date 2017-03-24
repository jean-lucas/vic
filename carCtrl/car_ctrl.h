#ifndef CAR_CTRL_H
#define CAR_CTRL_H



//constants
const int CAR_ID = 1;
const int OUTER_LANE = 2;
const int INNER_LANE = 1;


//method declarations
int init(int quickstart_mode);
int run();
void cleanup();
void pause_sys();



#endif


