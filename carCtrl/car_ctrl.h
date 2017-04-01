#ifndef CAR_CTRL_H
#define CAR_CTRL_H



//constants
const int CAR_ID = 1;


//system states
const int TEST_MODE  = 1; 
const int QUICK_START_MODE = 3;
const int BT_START_MODE    = 2;
const int SYSTEM_RUNNING = 4;





int car_start_mode;

//method declarations
int init(int quickstart_mode);
int run();
void cleanup();
void pause_sys();



#endif


