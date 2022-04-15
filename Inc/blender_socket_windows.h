#ifndef BLENDER_SOCKET_H
#define BLENDER_SOCKET_H
#include "mathlib.h"
//Network stuff
#include<winsock2.h>

int init_socket(void);
void send_to_blender(vec3 pos,vec3 E, vec4 pwm);
void close_socket(void);

#endif
