#include "mathlib.h"
#include "blender_socket.h"

//Standard library
#include <strings.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

//int sid, cid;

SOCKET sid;
SOCKET cid;

typedef struct state{
    vec3 pos;
    vec3 E;
    vec4 pwm;
}state;

int init_socket(void)
{

	WSADATA wsa;
	printf("\nInitialising Winsock...");
	if (WSAStartup(MAKEWORD(2,2),&wsa) != 0)
	{
		printf("Failed. Error Code : %d",WSAGetLastError());
		exit(EXIT_FAILURE);
	}

	struct sockaddr_in serverAddr,clientAddr;
	if((sid = socket(AF_INET,SOCK_STREAM,0))<0)
		printf("socket failed\n");

	//bzero(&serverAddr,sizeof(serverAddr));
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_port = htons(50001);
	serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1");

	if(bind(sid, (struct sockaddr*)&serverAddr,sizeof(serverAddr)) != 0){
		printf("could not bind\n");
		return -1;
	} else {
		printf("socket bound\n");
	}
	if((listen(sid,5)) != 0){
		printf("listen failed\n");
		return -1;
	} else {
		printf("Listened\n");
	}

	int sz = sizeof(clientAddr);
	cid = accept(sid,(struct sockaddr*)&clientAddr,&sz);
	if(cid < 0){
		printf("accept failed\n");
		return -1;
	} else {
		printf("Accepted\n");
	}
	return 1;
}

void send_to_blender(vec3 pos,vec3 E, vec4 pwm)
{
	char mess[10];
	state s = {{0,0,0},{0,0,0}};
    s.pos.x = pos.x;
    s.pos.y = pos.y;
    s.pos.z = pos.z;
    s.E.x = E.x;
    s.E.y = E.y;
    s.E.z = E.z;
    s.pwm.a[0] = pwm.a[0];
    s.pwm.a[1] = pwm.a[1];
    s.pwm.a[2] = pwm.a[2];
    s.pwm.a[3] = pwm.a[3];
    recv(cid,mess,sizeof(mess),0);
    //write(cid,&s,sizeof(s));
    send(cid,&s,sizeof(s),0);
}

void close_socket(void)
{
	char mess[] = {'C','l','o','s','e','\0'};
	send(cid,mess,sizeof(mess),0);
	closesocket(sid);
	WSACleanup();
}
