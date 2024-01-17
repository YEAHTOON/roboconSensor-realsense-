#ifndef __COMMON_H_
#define __COMMON_H_

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

//数据
template <class data_type>
class DATA
{
public:
    //初始化
    DATA(data_type *ptr, int size)
    {
        this->ptr = ptr;
        this->size = size;
    }

    //无输入初始化
    DATA(void)
    {
        this->ptr = nullptr;
        this->size = 0;
    }

    //设置一段数据
    void set_data(data_type *ptr, int size)
    {
        if(this->ptr == nullptr)
        {
            this->ptr = ptr;
            this->size = size;
        }
        else
        {
            this->free_data();
            this->size = 0;
        }
    }

    //获得数据指针
    data_type *getDataPtr(void)
    {
        if(this->ptr == nullptr)
        {
            perror("return empty data!");
        }
        else
        {
            return this->ptr;
        }
    }

    //获得数据大小
    int getSize(void)
    {
        if(this->size == 0)
        {
            perror("return empty data!");
        }
        else
        {
            return size;
        }
    }

    //释放数据空间
    void free_data(void)
    {
        if(this->ptr != nullptr)
        {
            free(this->ptr);
            this->ptr = nullptr;
            this->size = 0;
        }
        else
        {
            perror("free empty data!");
        }
    }

private:
    data_type *ptr;
    int size;
};


class communication
{
public:
    virtual void setparam(){}
    virtual void send(){}
    virtual void closeCommu(){}

private:   

};


#endif
