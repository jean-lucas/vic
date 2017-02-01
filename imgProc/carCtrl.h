#ifndef CARCTRL_H    
#define CARCTRL_H



//constants







//bluetooth request structure
struct btRequest {
	char serverAddr[18];  // "AB:12:CD:34:EF:56"
	int socket;
	char* message;

};


//bluetooth response structure 
struct btResponse {
	char senderAddr[18];
	char* message;
};





#endif