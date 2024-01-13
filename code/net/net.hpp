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
#include <sys/types.h>
#include <sys/msg.h>
#include <sys/ipc.h>

#include <mutex>

#define serveIP     "192.168.3.107"
#define servePort   3000

#define test_plane_distance 6.66f
#define test_plane_angle    7.77f

typedef struct dataToSend
{
    float plane_distance;       //平面距离
    float plane_angle;          //与平面角度
} dataToSend;

typedef struct netStatus
{
    uint8_t newRequest;
    uint16_t workStatus;
} netStatus;

typedef struct dataFromTCP
{
    int pipeline_read[10][2];
    int pipeline_write[10][2];
    int valid_pipe[10];
    int childcount;
} dataFromTCP;


#define mainStatusMsg   1
#define planeData       2
#define SL_data         3
typedef struct MainStatusMsg
{
    long int mtype;
    // uint16_t mainStatus;
    char mtext[1];
} MainStatusMsg;

// typedef struct pipeCouple
// {
//     int readpipe[2];
//     int writepipe[2];
// } pipeCouple;

void *processTCP(void *args);

#endif
