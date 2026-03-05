#include <stdio.h>
#include <stdlib.h>     
#include <string.h>
#include <string>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <iostream>

using namespace std;

int sockfd, newsockfd, portno;

void loop(){
	char buffer[256];
	while(1){
		bzero(buffer,256);
		int n=read(newsockfd, buffer, 255);
		if(n<0)
			{
			perror("ERROR reading from socket");
		}
		if (n>1)
			cout<<buffer;
	}
}

void error(char *msg)
{
perror(msg);
exit(1);
}

int main(int argc, char *argv[])
{


//unsigned clilen;
socklen_t clilen;
struct sockaddr_in serv_addr, cli_addr;
int n;

if (argc < 2)
{
fprintf(stderr,"ERROR, no port provided\n");
exit(1);
}

sockfd = socket(AF_INET, SOCK_STREAM, 0);

if (sockfd < 0)
{
error("ERROR opening socket");
}

bzero((char *) &serv_addr, sizeof(serv_addr));

portno = atoi(argv[1]);

serv_addr.sin_family = AF_INET;
serv_addr.sin_addr.s_addr = INADDR_ANY;
serv_addr.sin_port = htons(portno);

if (bind(sockfd, (struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
{
error("ERROR on binding");
}

listen(sockfd,5);
clilen = sizeof(cli_addr);

newsockfd = accept(sockfd,(struct sockaddr *) &cli_addr, &clilen);

if (newsockfd < 0)
{
error("ERROR on accept");
}

thread usbl_loop(loop);

	while(1){
		string test;
		getline(cin,test);
		//test+='\n';
		int n1 =write(newsockfd,test.c_str(),test.length());
	}	

return 0;
}
