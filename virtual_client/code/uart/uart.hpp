#ifndef __UART_HPP
#define __UART_HPP

#include "../common/common.hpp"
#include <termios.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>


//小摄像头需要的数据
typedef struct smallCamera_data
{
    float distance;
    float angle;
} smallCamera_data;


//把数据转换成一个uint8类型的数组
template<class T>
class data_inUint8
{
public:
    data_inUint8(T input)
    {
        uint8_t *temp = (uint8_t *)malloc(sizeof(input));
        memcpy(temp, &input, sizeof(input));
        this->addr = temp;
        this->size = sizeof(input);
    }

    ~data_inUint8(void)
    {
        if(this->addr != nullptr)
        {
            free(this->addr);
            this->addr = nullptr;
        }
    }

    uint8_t *getAddr(void)
    {
        return this->addr;
    }

    size_t getSize(void)
    {
        return this->size;
    }

private:
    uint8_t *addr;
    size_t size;
};


//每个包的包头
class uartPack_head
{
public:
    uartPack_head(uint8_t head_1, uint8_t head_2)
    {
        this->data[0] = head_1;
        this->data[1] = head_2;
    }

    uint8_t getData(int index)
    {
        return this->data[index];
    }

private:
    uint8_t data[2];
};



//每个包的id
class uartPack_ID
{
public:
    uartPack_ID(uint8_t data)
    {
        this->data = data;
    }

    uint8_t getData(void)
    {
        return this->data;
    }

private:
    uint8_t data;
};



//每个包的校验位
class uartPack_check
{
public:
    uartPack_check(uint8_t data)
    {
        this->data = data;
    }

    uint8_t getData(void)
    {
        return this->data;
    }

private:
    uint8_t data;
};



//每个包的包尾
class uartPack_tail
{
public:
    uartPack_tail(uint8_t tail_1, uint8_t tail_2)
    {
        this->data[0] = tail_1;
        this->data[1] = tail_2;
    }

    uint8_t getData(int index)
    {
        return this->data[index];
    }

private:
    uint8_t data[2];
};



/**
 * 串口通信的基本单位
 * 需要释放空间，保护内存
*/
template <class T>
class uartPack
{
public:

    uartPack(uartPack_head head, uartPack_ID id, uartPack_tail tail, uartPack_check check, data_inUint8<T> data_toBeSent)
    {
        //串口包存放的地址
        uint8_t *temp = (uint8_t *)malloc(data_toBeSent.getSize() + 6);

        temp[0] = head.getData(0);
        temp[1] = head.getData(1);
        temp[2] = id.getData();
        memcpy(&temp[3], data_toBeSent.getAddr(), data_toBeSent.getSize());
        temp[data_toBeSent.getSize() + 3] = check.getData();
        temp[data_toBeSent.getSize() + 4] = tail.getData(0);
        temp[data_toBeSent.getSize() + 5] = tail.getData(1);

        this->addr = temp;
        this->size = data_toBeSent.getSize() + 6;
    }

    ~uartPack(void)
    {
        if(this->addr != nullptr)
        {
            free(this->addr);
            this->addr = nullptr;
        }
    }

    //包的地址
    uint8_t *getAddr(void)
    {
        return this->addr;
    }

    //获得大小
    size_t getSize(void)
    {
        return this->size;
    }

private:
    uint8_t *addr;
    size_t size;
};


/**
 * 串口通信
*/
class uart : communication
{
public:
    uart(const char *filename, int baudrate)
    {
        if(filename != nullptr)
        {
            this->fileDes = open(filename, O_RDWR|O_NOCTTY|O_NDELAY);
            this->baudrate = baudrate;
        }
    }

    //设置参数
    virtual void setparam(void)
    {
        //串口参数
        struct termios options;
        memset(&options, 0, sizeof(options));

        //设置串口参数后应用
        tcgetattr(this->fileDes, &options);
        options.c_cflag |= (CLOCAL|CREAD );     //CREAD 开启串行数据接收，CLOCAL并打开本地连接模式
        options.c_cflag &= ~CSIZE;              //先使用CSIZE做位屏蔽
        options.c_cflag |= CS8;                 //设置8位数据位
        options.c_cflag &= ~PARENB;             //无校验位
        options.c_cflag &= ~CSTOPB;
        options.c_cc[VTIME] = 0;
        options.c_cc[VMIN]  = 0;
        cfsetispeed(&options, this->baudrate);
        cfsetospeed(&options, this->baudrate);
        tcsetattr(this->fileDes, TCSANOW, &options);

        //清空缓冲区数据与请求
        tcflush(this->fileDes ,TCIFLUSH);
    }

    virtual void send(uint8_t *addr, size_t size)
    {   
        if(addr != nullptr)
        {
            //传出
            write(fileDes, addr, size);
        }
    }

    virtual void closeCommu(void)
    {
        close(fileDes);
    }

private:
    int fileDes;            //文件描述符
    int baudrate;           //波特率
};


uint8_t getFloat_lastEightBits(float theF);

#endif
