
#include "utilities_u.h"

int main(void)
{
    unsigned char messagesize = 0;	//size of message to send
	unsigned char buffermessage[263];//message to send
	unsigned char payload[255];//payload message
	payload[0]= 1;//payload command
	payload[1]=0; //lunghezza payload
	messagesize=createRequestMessage(buffermessage,0,1,payload,2);
    return 0;
}
