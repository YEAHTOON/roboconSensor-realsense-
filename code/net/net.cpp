#include "net.hpp"

/**
 * 获得用于监听的套接字
*/
int getListenSocket(void)
{
    //本地地址
    struct sockaddr_in serAddr;
    serAddr.sin_addr.s_addr     = inet_addr(serveIP);   //服务器ip地址
    serAddr.sin_port            = htons(servePort);     //服务器端口号
    serAddr.sin_family          = AF_INET;              //ipv4协议

    //创建用于监听的套接字
    int listenSocket = socket(AF_INET, SOCK_STREAM, 0);
    if(listenSocket < 0) {perror("socket created failed"); exit(1); }

    //设置网络接口为以太网
    // const char *theEth;
    // theEth = "enx207bd2bc8361";
    // setsockopt(listenSocket, SOL_SOCKET, SO_BINDTODEVICE, theEth, sizeof(theEth));

    //设置为端口可复用，否则关闭程序后再连会报错
    int ifReuse = 1;
    setsockopt(listenSocket, SOL_SOCKET, SO_REUSEADDR, &ifReuse, sizeof(ifReuse));

    //电脑套接字绑定实际地址
    int bindRet = bind(listenSocket, (struct sockaddr*)&serAddr, sizeof(serAddr));
    if(bindRet < 0) {perror("bind failed"); exit(1); }

    return listenSocket;
}




/**
 * 接收请求的线程
*/
typedef struct receiveData_args
{
    int socketNum;
    char *request;
    uint8_t *status_addr;
} receiveData_args;
void *receiveData(void *args)
{
    //获得传入参数
    receiveData_args *Args = (receiveData_args *)args;

    while(1)
    {
        recv(Args->socketNum, (void *)(Args->request), 128, 0);
        std::cout << Args->request << std::endl;
    }
}



/**
 * 响应客户端的请求
*/
void Respond_ClientRequst(int socketNum, struct sockaddr_in &cliAddr, socklen_t &cliLen)
{
    //发送信息的状态
    uint8_t status = 0;

    //字符串请求
    char request[128];

    //创建线程处理收发任务
    pthread_t recv_pid;

    //接收请求的线程
    receiveData_args receiveData_args_input;
    receiveData_args_input.socketNum    = socketNum;
    receiveData_args_input.request      = request;
    pthread_create(&recv_pid, NULL, receiveData, (void *)&receiveData_args_input);

    //发送数据
    while(1)
    {
        sleep(10);
    }
}





/**
 * 处理接收到的tcp通信
*/
void *processTCP(void *args)
{
    //杀死退出的子进程，预防僵尸进程
    signal(SIGCHLD, SIG_IGN);

    //用于监听的套接字
    int listenSocket = getListenSocket();

    //监听端口
    listen(listenSocket, 10);

    while(1)
    {
        struct sockaddr_in cliAddr;                    //暂时存储来自客户端的地址
        socklen_t cliLen;                              //暂时存储来自客户端的地址的长度

        int each_clientSocket = accept(listenSocket, (struct sockaddr*)&cliAddr, &cliLen);       //每一个客户端发来的tcp请求
        std::cout << "success" << std::endl;

        //创建子进程
        if(!fork())
        {
            prctl(PR_SET_PDEATHSIG, SIGKILL);                               //子进程接收父进程的退出消息，自行在父进程退出后退出
            Respond_ClientRequst(each_clientSocket, cliAddr, cliLen);       //响应客户端的请求
        }
    }
}
