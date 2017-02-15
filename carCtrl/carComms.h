#ifndef CARCOMMS_H
#define CARCOMMS_H


/* Constants */
const char[18] IC_ADDR = "AC:2B:6E:04:BF:27";
const char[18] VC_ADDR = "XX:XX:XX:XX:XX:XX";

const int IC_PORT = 1337;

/* function declarations */
int sendToIC(char* msg);
int recvFromIC();


#endif
