#include <stdio.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <string.h>
#include <iostream>
#include <signal.h>
#include <sys/prctl.h>
#include <pthread.h>
#include "uart/uart.hpp"

// int main(int argc, char **argv)
// {
//     //防止僵尸进程
//     signal(SIGCHLD, SIG_IGN);

//     //小摄像头的数据
//     smallCamera_data smallCamera_data_1;
//     smallCamera_data_1.angle = 6.66f;
//     smallCamera_data_1.distance = 7.77f;

//     //包头、包尾、校验位、获得无符号八位形式的数据
//     data_inUint8<smallCamera_data> uint8_data(smallCamera_data_1);
//     uartPack_head theHead(0xFF, 0xFE);
//     uartPack_ID theID(1);
//     uartPack_tail theTail(0x0A, 0x0D);
//     uartPack_check theCheck(getFloat_lastEightBits(smallCamera_data_1.angle + smallCamera_data_1.distance));

//     //创建要发送的包
//     uartPack<smallCamera_data> thePack(theHead, theID, theTail, theCheck, uint8_data);

//     //创建串口通信
//     uart theUART("", B115200);
//     theUART.setparam();

//     while(1)
//     {
//         theUART.send(thePack.getAddr(), thePack.getSize());
//         sleep(1);
//     }

//     delete &uint8_data;
//     delete &thePack;

//     theUART.closeCommu();

//     return 0;
// }






//接收数据线程
void *recvde(void *args)
{
    char buff[128];
    while(1)
    {
        recv(*((int *)args), (void *)buff, 128, MSG_WAITFORONE);
        std::cout << buff << "\n" << std::endl;
        memset(buff, 0, strlen(buff));
        usleep(10000);
    }
}



int main(int argc, char **argv)
{
    //防止僵尸进程
    signal(SIGCHLD, SIG_IGN);

    // std::cout << typeid(argv[1]).name() << std::endl;

    //本地的地址
    struct sockaddr_in cliAddr;
    cliAddr.sin_addr.s_addr = inet_addr("192.168.3.112");
    cliAddr.sin_port = htons(std::atoi(argv[1]));
    cliAddr.sin_family = AF_INET;

    //套接字
    int pcSocket = socket(AF_INET, SOCK_STREAM, 0);
    if(pcSocket < 0) {perror("socket created failed"); exit(1); }

    //绑定本地地址
    int bindRet = bind(pcSocket, (struct sockaddr*)&cliAddr, sizeof(cliAddr));
    if(bindRet < 0) {perror("bind failed"); exit(1); }

    //设置网络接口为以太网
    // const char *theEth;
    // theEth = "enx207bd2bc8361";
    // setsockopt(pcSocket, SOL_SOCKET, SO_BINDTODEVICE, theEth, sizeof(theEth));

    //设置为可复用，否则关闭程序后再连会报错
    int on = 1;
    setsockopt(pcSocket, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

    //另一端的地址
    struct sockaddr_in serAddr;
    serAddr.sin_addr.s_addr = inet_addr("192.168.3.209");
    serAddr.sin_port = htons(3000);
    serAddr.sin_family = AF_INET;

    //尝试连接
    int connectRet = connect(pcSocket, (struct sockaddr*)&serAddr, sizeof(serAddr));
    if(connectRet < 0) {perror("connect failed"); exit(1); }
    std::cout << "success" << std::endl;

    pthread_t pid;
    pthread_create(&pid, NULL, recvde, &pcSocket);

    //发送消息
    std::string input;
    char buff[128];
    while(1)
    {
        //缓冲区
        char sendbuff1[10] = {0};
        char sendbuff2[10] = {0};

        //读取输入
        scanf("%s%s", sendbuff1, sendbuff2);

        //字符串拼接
        std::string sendbuff1_s = sendbuff1;
        std::string sendbuff2_s = sendbuff2;
        std::string sendbuff_s = sendbuff1_s + " " + sendbuff2_s;
        const char *sendbuff = sendbuff_s.c_str();

        //发出消息
        send(pcSocket, (void *)sendbuff, strlen(sendbuff) + 1, 0);
        // recv(pcSocket, (void *)buff, 128, 0);

        //打印返回信息
        // std::cout << "back: " << buff << "\n" << std::endl;
    }

    // close(pcSocket);
}


// int main(int argc, char **argv)
// {
//     signal(SIGCHLD, SIG_IGN);

//     while(1)
//     {
//         std::cout << "alright" << std::endl;
//         sleep(1);
//     }

//     return 0;
// }

