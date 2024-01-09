#ifndef __NET_H_
#define __NET_H_

#include <stdio.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <string.h>
#include <sys/prctl.h>
#include <signal.h>
#include <pthread.h>
#include <iostream>

#define serveIP     "127.0.0.1"
#define servePort   3000

#define test_plane_distance 6.66f
#define test_plane_angle    7.77f

typedef struct dataToSend
{
    float plane_distance;       //平面距离
    float plane_angle;          //与平面角度
} dataToSend;

void *processTCP(void *args);

#endif
