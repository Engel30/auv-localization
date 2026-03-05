/*
    Simple udp client che prende il gps da seriale invia il GPGGA
*/
#include<stdio.h> //printf
#include<stdlib.h> //exit(0);
#include<arpa/inet.h>
#include<sys/socket.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <thread>
#include <string.h> 
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
 
#define SERVER "127.0.0.1"
#define BUFLEN 512  //Max length of buffer
#define PORT 8881   //The port on which to send data
#define BRATE B9600

using namespace std;


void gps_sring(char* gps,int readBytes,int fd_serial){//function to keep the entire gsp string
	if (gps[readBytes-1]!='\n'){
		char buffer[200];
		int readBytes2=0;
		usleep(10000);
		readBytes2=read( fd_serial, buffer, 200 );
		buffer[readBytes2]=0;
		strncat(gps,buffer,200);
		gps_sring(gps,readBytes+readBytes2,fd_serial);
	}
}


int setupSerial(int fd){//setup the serial port of gps 
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	//read structure
	if (tcgetattr (fd, &tty) != 0){
		printf ("error %d from tcgetattr\r\n", errno);
		return -1;
	}
	cfmakeraw(&tty);
	tty.c_iflag &= ~IGNBRK;         // disable break processing
	tty.c_lflag = 0;                // no signaling chars, no echo,
									// no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_iflag &= ~(ICRNL | IGNCR );
		
	tty.c_cc[VMIN]  = 0;            // read doesn't block
	tty.c_cc[VTIME] = 0;            // 0.5 seconds read timeout
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
									// enable reading(ICANON | ECHO | ECHOE | ISIG)
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= 0;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;
	cfsetospeed (&tty, (speed_t)BRATE);
	cfsetispeed (&tty, (speed_t)BRATE);	
	if (tcsetattr (fd, TCSANOW, &tty) != 0){
		printf ("error %d from tcsetattr\r\n", errno);
		return -1;
	}
	return 0;
}

int fd;

 
void die(char *s)
{
    perror(s);
    exit(1);
}
 
int main(void)
{

    struct sockaddr_in si_other;
    int s, i, slen=sizeof(si_other);
    char buf[BUFLEN];
    //char message[BUFLEN]="$GPGGA,134658.00,4106.9792,N,23402.3003,W,2,09,1.0,1048.47,M,-16.27,M,08,AAAA*60\n";
	char message[BUFLEN]="$GPGGA,160843.000,4339.8565,N,01537.5022,E,1,11,0.85,5.9,M,41.6,M,,*60\n";
    if ( (s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        die("socket");
    }
 
    memset((char *) &si_other, 0, sizeof(si_other));
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(PORT);
    if (inet_aton(SERVER , &si_other.sin_addr) == 0) 
    {
        fprintf(stderr, "inet_aton() failed\n");
        exit(1);
    }    
    string port="/dev/ttyUSB0";
	fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
	if(fd < 0) printf ("error opening serial\r\n");	
	if(setupSerial(fd)==0)
		printf ("Serial init finished, acting like MINICOM. press esc to quit\r\n");
	else{
		exit(1);
	} 
	
	bool a=true;
    while (1)
		{
		char buffer[500];
		int readBytes=read( fd, buffer, 500 );
		buffer[readBytes]=0;
		if(readBytes>0)
			{
			gps_sring(buffer,readBytes,fd);	
			char * line;
			string stringa;
			line = strtok (buffer,"\r\n");
			while (line != NULL)// first cicle to keep the initial position
				{
				stringa=line;
				line = strtok (NULL, "\r\n");			
				//send the message
				if (stringa.compare(0,6,"$GPGGA")==0){
					stringa+="\n";
					cout<<stringa.c_str()<<endl;
					//cout<<message;
					//if (sendto(s, message, strlen(message) , 0 , (struct sockaddr *) &si_other, slen)==-1)
						if (sendto(s, stringa.c_str(), stringa.length() , 0 , (struct sockaddr *) &si_other, slen)==-1)
						{
						die("sendto()");
						}
					}
				}
			} 
         usleep(20000);
		} 
    close(s);
    return 0;
} 
