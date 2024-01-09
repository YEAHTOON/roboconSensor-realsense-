#include <stdio.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <string.h>
#include <iostream>

int main(void)
{
    //本地的地址
    struct sockaddr_in cliAddr;
    cliAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
    cliAddr.sin_port = htons(2000);
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
    serAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
    serAddr.sin_port = htons(3000);
    serAddr.sin_family = AF_INET;

    //尝试连接
    int connectRet = connect(pcSocket, (struct sockaddr*)&serAddr, sizeof(serAddr));
    if(connectRet < 0) {perror("connect failed"); exit(1); }
    std::cout << "success" << std::endl;

    //发送消息
    const char *nihao = "nihao";
    while(1)
    {
        send(pcSocket, nihao, sizeof(nihao), 0);
        sleep(1);
    }
}
