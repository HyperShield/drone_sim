#include "mathlib.h"
#include "blender_socket.h"

//Standard library
#include <strings.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

//Network stuff
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

int sid, cid;

typedef struct state{
    vec3 pos;
    vec3 E;
    vec4 pwm;
}state;

int init_socket(void)
{
	struct sockaddr_in serverAddr,clientAddr;
	if((sid = socket(AF_INET,SOCK_STREAM,0))<0){
		printf("socket failed\n");
		return -1;
	} else {
		printf("Socket made\n");
	}

	bzero(&serverAddr,sizeof(serverAddr));
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

	unsigned int sz = sizeof(clientAddr);
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
	unsigned char mess[10];
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
    read(cid,mess,sizeof(mess));
    write(cid,&s,sizeof(s));
}

void close_socket(void)
{
	unsigned char mess[] = {'C','l','o','s','e','\0'};
	write(cid,mess,sizeof(mess));
	close(sid);
}
