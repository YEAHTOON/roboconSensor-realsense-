#include "uart.hpp"

//获取float类型数据的后八位
uint8_t getFloat_lastEightBits(float theF)
{
    uint8_t temp[4];
    uint8_t result;

    memcpy(temp, &theF, sizeof(theF));
    memcpy(&result, temp+3, sizeof(result));

    return result;
}
