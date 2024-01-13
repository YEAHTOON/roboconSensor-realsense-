#include "net.hpp"

netStatus NetStatus;
uint16_t status;
uint16_t temp_mainStatus;
int sokcetNumber;

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



/* ******************向客户端发送消息的线程********************************************************* */
/**
 * 发送平面的数据
*/
void *send_PL_data(void *args)
{
    int msqid = msgget(111, IPC_CREAT|400);
    struct Temp
    {
        long mtype;
        float mtext[4];
    } temp;

    while(1)
    {
        //接收消息
        msgrcv(msqid, &temp, sizeof(temp.mtext), 2, 0);

        //数据全部转换成字符串
        // std::string PL_distance = std::to_string(temp.mtext[0]);
        // std::string PL_direction_x = std::to_string(temp.mtext[1]);
        // std::string PL_direction_y = std::to_string(temp.mtext[2]);
        // std::string PL_direction_z = std::to_string(temp.mtext[3]);

        char PLDistance[20], PLDirection[20];
        sprintf(PLDistance,"%f",temp.mtext[0]);
        sprintf(PLDirection,"%f",temp.mtext[1]);
        strcat(PLDistance,",");
        strcat(PLDistance,PLDirection);

        //拼接
        // std::string toSend = "plane distance: " + PL_distance + "\n" + "plane x,y,z: " + PL_direction_x + "," + PL_direction_y + "," + PL_direction_z + ",";
        // std::string toSend = PL_distance + "," + PL_direction_x;
        // std::string toSend = "0.25,0.34";
        // char *toSend_c = (char *)toSend.c_str();

        // std::cout << toSend_c << std::endl;

        // 发送
        send(sokcetNumber, (void *)PLDistance, strlen(PLDistance), 0);


        // send(sokcetNumber, (void *)"1", 1, 0);
        // sleep(1);
    }
}


/**
 * 发送直线的数据
*/
void *send_SL_data(void *args)
{
    int msqid = msgget(222, IPC_CREAT|400);
    struct Temp
    {
        long mtype;
        float mtext[4];
    } temp;

    while(1)
    {
        send(sokcetNumber, "pldata", sizeof("pldata"), 0);
        sleep(1);
    }
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
        //将缓冲区清零
        // char temp[128] = {0};
        // memcpy(Args->request, temp, 128);

        //接收到新的请求
        ssize_t recvRet = recv(Args->socketNum, (void *)(Args->request), 128, MSG_WAITFORONE);
        NetStatus.newRequest = 1;

        //返送给客户端确定
        // ssize_t sendRet = send(Args->socketNum, (void *)(Args->request), 128, 0);
    }
}

//向客户端传输的两个线程
pthread_t send_pid[2];

/**
 * 翻译新的请求
*/
void Check_NewRequest(std::string theRequest ,int writepipe[2], int socketNum)
{
    //分割字符串
    char *rest_string = (char *)theRequest.c_str();     //用于递归中保存余下未分割的字符串
    char theRequest_c[24] = {0};
    memcpy(theRequest_c, rest_string, 20);
    const char split_symbol = ' ';                      //分割的标志
    char *tempString;

    //指令请求
    tempString = strtok_r((char *)rest_string, &split_symbol, &rest_string);
    if(strstr(tempString, "create") || strstr(tempString, "shutdown"))
    {
        //给父进程发送消息
        write(writepipe[1], (char *)theRequest_c, sizeof(theRequest_c));

        //接收父进程消息确定新mainStatus的值
        int msqid = msgget(12345, IPC_CREAT|400);
        struct Temp
        {
            long mtype;
            uint16_t mtext;
        } temp;
        msgrcv(msqid, &temp, sizeof(temp.mtext), mainStatusMsg, 0);
        temp_mainStatus = temp.mtext;
    }

    else
    {
        if(strstr(theRequest_c, "start PL"))
        {
            std::cout << theRequest_c << std::endl;
            if((NetStatus.workStatus & 0x08) == 0x00)
            {
                std::cout << "ok" << std::endl;
                //创建线程
                pthread_create(send_pid, NULL, send_PL_data, NULL);

                //改变状态
                NetStatus.workStatus |= 0x08;
            }
        }
        else if(strstr(theRequest_c, "start SL"))
        {
            if((NetStatus.workStatus & 0x04) == 0x00)
            {
                //创建线程
                pthread_create(send_pid+1, NULL, send_SL_data, NULL);

                //改变状态
                NetStatus.workStatus |= 0x04;
            }
        }
        else if(strstr(theRequest_c, "stop PL"))
        {
            if((NetStatus.workStatus & 0x08) == 0x08)
            {
                pthread_cancel(send_pid[0]);
                NetStatus.workStatus &= 0xF7;
            }
        }
        else if(strstr(theRequest_c, "stop SL"))
        {
            if((NetStatus.workStatus & 0x04) == 0x04)
            {
                pthread_cancel(send_pid[1]);
                NetStatus.workStatus &= 0xFB;
            }
        }
    }

    status = temp_mainStatus | NetStatus.workStatus;

    // while(NULL != ( tempString = strtok_r((char *)rest_string, &split_symbol, &rest_string)))       //循环分割
    // {
        
    // }
}




/**
 * 响应客户端的请求
*/
void Respond_ClientRequst(int socketNum, struct sockaddr_in &cliAddr, socklen_t &cliLen, int readpipe[2], int writepipe[2])
{
    //得到消息队列
    int msqid = msgget(12345, IPC_CREAT|400);

    //字符串请求
    char request[128] = {0};

    //创建线程处理收发任务
    pthread_t recv_pid;

    //接收请求的线程
    receiveData_args receiveData_args_input;
    receiveData_args_input.socketNum    = socketNum;
    receiveData_args_input.request      = request;
    sokcetNumber = socketNum;
    pthread_create(&recv_pid, NULL, receiveData, (void *)&receiveData_args_input);

    //发送数据（主线程）
    while(1)
    {
        //有新的请求
        if(NetStatus.newRequest)
        {
            NetStatus.newRequest = 0;   //置零

            //请求字符串
            std::string Request = request;

            //更新
            char temp[128] = {0};
            memcpy(request, temp, 128);

            //检查新的请求，改变工作状态
            Check_NewRequest(Request, writepipe, socketNum);
        }

        //接收主进程的状态
        // uint16_t mainStatus = 0x00;
        // std::cout << read(readpipe[0], &mainStatus, sizeof(mainStatus)) << std::endl;
        // std::cout << mainStatus << std::endl;

        //发送消息
        // send(socketNum, "nihao", sizeof("nihao"), 0);

        // std::cout << status << std::endl;

        usleep(10000);
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

    //管道地址传到main
    dataFromTCP *sendToMain = (dataFromTCP *)args;

    int childcount = 0;
    while(1)
    {
        struct sockaddr_in cliAddr;                    //暂时存储来自客户端的地址
        socklen_t cliLen;                              //暂时存储来自客户端的地址的长度

        int each_clientSocket = accept(listenSocket, (struct sockaddr*)&cliAddr, &cliLen);       //每一个客户端发来的tcp请求

        //创建管道，为后续通信做准备
        pipe(sendToMain->pipeline_read[childcount]);
        pipe(sendToMain->pipeline_write[childcount]);
        sendToMain->valid_pipe[childcount] = 1;

        //创建子进程
        if(!fork())
        {
            prctl(PR_SET_PDEATHSIG, SIGKILL);       //子进程接收父进程的退出消息，自行在父进程退出后退出
            
            close(sendToMain->pipeline_read[childcount][0]);        //关闭读端
            close(sendToMain->pipeline_write[childcount][1]);       //关闭写端    

            //初始化一些全局参数
            NetStatus.newRequest = 0;
            NetStatus.workStatus = 0x00;      
            temp_mainStatus = 0x00;
            status = 0x00;
            sokcetNumber = 0;

            //响应客户端的请求
            Respond_ClientRequst(each_clientSocket, cliAddr, cliLen, sendToMain->pipeline_write[childcount], sendToMain->pipeline_read[childcount]);
        }
        else
        {
            close(sendToMain->pipeline_read[childcount][1]);        //关闭写端
            close(sendToMain->pipeline_write[childcount][0]);       //关闭读端
        }

        //新管道
        childcount++;

        sendToMain->childcount = childcount;
    }
}
