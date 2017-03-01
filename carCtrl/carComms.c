/*
	http://people.csail.mit.edu/albert/bluez-intro
*/

#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>

#include "carComms.h"

/* 
    Send a message over Bluetooth to the Intersection Controller
    Input:  msg to send
    Output: status of send (negative for error,  0 for success)
*/
int sendToIC(char* msg) {

    struct sockaddr_rc addr = { 0 };
    int s, status;

    // allocate a socket
    s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

    // set the connection parameters (who to connect to)
    addr.rc_family = AF_BLUETOOTH;
    addr.rc_channel = (uint8_t) 1;
    str2ba( IC_ADDR, &addr.rc_bdaddr );

    // connect to server
    status = connect(s, (struct sockaddr *)&addr, sizeof(addr));

    // send a message
    if (status == 0) {
        status = send(s, msg, sizeof(msg), MSG_DONTWAIT);
    }

    //error sending the message, try again next time.
    if (status < 0) {
        printf("oh nos\n");
    }

    close(s);
    return status;
}

/*
    Try to receive a message from the Intersection Controller via Bluetooth
    Input: ??
    Output: result from IC (negative for error, 0 for stop @ intersection, 1 to proceed)
*/

/*
int recvFromIC() {

    struct sockaddr_rc loc_addr = { 0 }, rem_addr = { 0 };
    char buf[1024] = { 0 };
    int s, client, bytes_read;
    socklen_t opt = sizeof(rem_addr);

    // allocate socket
    s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

    // bind socket to port 1 of the first available 
    // local bluetooth adapter
    loc_addr.rc_family = AF_BLUETOOTH;
    loc_addr.rc_bdaddr = *BDADDR_ANY;
    loc_addr.rc_channel = (uint8_t) 1;
    bind(s, (struct sockaddr *)&loc_addr, sizeof(loc_addr));

    //listen with a backlog of 1
    listen(s, 1);

    // accept one connection
    client = accept(s, (struct sockaddr *)&rem_addr, &opt);

    ba2str( &rem_addr.rc_bdaddr, buf );
    fprintf(stderr, "accepted connection from %s\n", buf);
    memset(buf, 0, sizeof(buf));

    // read data from the client
    bytes_read = read(client, buf, sizeof(buf));
    if( bytes_read > 0 ) {
        printf("received [%s]\n", buf);
    }

    // close connection
    close(client);
    close(s);
    return 0;

}
*/