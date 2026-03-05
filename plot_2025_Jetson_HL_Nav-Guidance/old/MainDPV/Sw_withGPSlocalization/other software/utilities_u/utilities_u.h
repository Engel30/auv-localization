
#include<sys/types.h>

#ifndef UTILITIES_H_
#define UTILITIES_H_

#define BROADCAST 0xF
#define MAXADDRESS 0x4F
#define MAXNUMDEVICES 32

struct stateMachine {
	int state;		//stato
	unsigned char message[263];	//messaggio
	int pointer;	//posizione carattere che si sta processando
	int counter;	//contatore
	unsigned char dataLength;
	unsigned char totalLength;
	unsigned char type;
	unsigned char address;
	unsigned char vendor;
	unsigned char model;
	unsigned char command;
}; 

struct unit {
	unsigned char type;
	unsigned char address;
	unsigned char vendor;
	unsigned char model;
};

struct device {
	unsigned char ta;
	unsigned char vm;
};

extern char executeCheckMessage(struct stateMachine* s, struct unit* u, char genericList[], int genericListLength, char specialList[], int specialListLength);
extern unsigned char createMessage(unsigned char buffer[],struct unit* u,struct stateMachine* s,unsigned char payload[], unsigned char payloadLen, unsigned char c);
extern char executeStateMachine(unsigned char c, struct stateMachine *s,unsigned char mode);
extern unsigned char createRequestMessage(unsigned char messageToSend[], unsigned char type, unsigned char address, unsigned char payload[], unsigned char payloadLen);


#endif /* UTILITIES_H_ */
