
#include "utilities_u.h"

char executeStateMachine(unsigned char c, struct stateMachine *s, unsigned char mode)	//mode=0 request, mode=1 answer
{
	char ret = 0;
	switch (s->state)
	{
	case 0:
		//controllo Header
		s->pointer = 0;
		if((mode == 0 && c == 0xFA) || (mode == 1 &&  c== 0xFD))	//check if it is a request or an answer
		{
			s->state = 1;
			s->counter = 0;
			s->message[s->pointer]=c;
		}
		break;
	case 1:
		//copia Header fino a checksum
		if(s->counter == 4) s->state = 2;
		s->counter++;
		s->message[s->pointer]=c;
		break;
	case 2:
	{
		//calcolo checksum
		unsigned char checksum = 0;
		unsigned char i;
		for(i=0;i<6;i++) checksum^=s->message[i];
		if((checksum == c) && ((s->message[1] == 0xAF && mode == 0)||(s->message[1] == 0xDF && mode == 1)))
		{
			s->state = 3;
			s->counter = 0;
			s->dataLength = s->message[5];
			s->message[s->pointer]=c;
		} else s->state = 0;
		break;
	}
	case 3:
		//copia payload data
		if(s->counter == s->dataLength-1) s->state = 4;
		s->counter++;
		s->message[s->pointer]=c;
		break;
	case 4:
	{
		//calcolo totalChecksum
		unsigned char totalChecksum=0;
		unsigned char i;
		for(i=0;i<s->pointer;i++) totalChecksum^=s->message[i];
		if(totalChecksum == c)
		{
			s->totalLength = s->pointer+1;
			s->message[s->pointer]=c;
			s->message[s->pointer+1]=0x00;
			ret=1;
			//
			s->type = s->message[2]>>4;
			//s->address = s->message[2]&0x0F;
			//s->vendor = s->message[3]>>4;
			//s->model = s->message[3]&0x0F;
			s->vendor = s->message[2]&0x0F;
			s->model = s->message[3]>>4;
			s->address = s->message[3]&0x0F;
			s->command = s->message[7];

		}
		s->state = 0;
		break;
	}
	}
	s->pointer++;
	return ret;
}

char executeCheckMessage(struct stateMachine *s, struct unit *u, char genericList[], int genericListLength, char specialList[], int specialListLength)
{
	
	int i=0;
	if((s->type == u->type) && ((s->address == u->address)||(s->address == BROADCAST)))	//message is for me
	{
		
		for(i=0; i<genericListLength;i++)	if(genericList[i]==s->command) return 1; //command is generic
		for(i=0; i<specialListLength;i++) 	if(specialList[i]==s->command && ((s->vendor == u->vendor) && (s->model == u->model))) return 1;
	}
	return 0;
}

unsigned char createMessage(unsigned char buffer[],struct unit* u,struct stateMachine* s,unsigned char payload[], unsigned char payloadLen, unsigned char c){
	//create an ANSWER message
	int i =0;
	unsigned char checksum=0;
	buffer[0]=0xFD;
	buffer[1]=0xDF;
	//buffer[2]=u.type<<4 | (u.address & 0x0F);
	//buffer[3]=u.vendor<<4 | (u.model & 0x0F);
	buffer[2]=u->type<<4 | (u->vendor & 0x0F);
	buffer[3]=u->model<<4 | (u->address & 0x0F);
	buffer[4]=0x00;
	buffer[5]=payloadLen + s->dataLength +3;	//MSB and LSB
	for(i=0;i<6;i++) checksum^=buffer[i];
	buffer[6]=checksum;
	for(i=0;i<s->dataLength;i++) buffer[7+i]=s->message[7+i];
	unsigned char msb = (unsigned char) ((c>>8) & 0xFF);	//Calculate MSB
	unsigned char lsb = (unsigned char) (c & 0xFF);	//Calculate LSB
	buffer[7+s->dataLength]=msb;
	buffer[8+s->dataLength]=lsb;
	buffer[9+s->dataLength]=payloadLen;
	for(i=0;i<payloadLen;i++) buffer[10+s->dataLength+i]=payload[i];
	checksum=0;
	for(i=0;i<10+s->dataLength+payloadLen;i++) checksum^=buffer[i];
	buffer[10+s->dataLength+payloadLen]=checksum;
	return 11+s->dataLength+payloadLen;
}

unsigned char createRequestMessage(unsigned char messageToSend[], unsigned char type, unsigned char address, unsigned char payload[], unsigned char payloadLen)
{
	//create a REQUEST message
	int i =0;
	unsigned char checksum=0;
	messageToSend[0]=0xFA;
	messageToSend[1]=0xAF;
	messageToSend[2]=type<<4 | (0x00 & 0x0F);//(address & 0x0F);
	messageToSend[3]=0x00<<4 | (address & 0x0F);	//VENDOR E MODEL devono essere formalizzate
	messageToSend[4]=0x00;
	messageToSend[5]=payloadLen;
	for(i=0;i<6;i++) checksum^=messageToSend[i];
	messageToSend[6]=checksum;
	checksum=0;
	for(i=0;i<payloadLen;i++) {
		messageToSend[7+i]=payload[i];
		checksum^=payload[i];
	}
	//for(i=0;i<7+payloadLen;i++) checksum^=messageToSend[i];
	messageToSend[7+payloadLen]=checksum;
	return 8+payloadLen;
}



