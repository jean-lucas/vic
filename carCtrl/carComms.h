#ifndef CARCOMMS_H
#define CARCOMMS_H



const int IC_PORT = 3;

/* function declarations */
char* serializeMessage();
int sendToIC(char* msg);
void* recvFromIC(void* c);


#endif

