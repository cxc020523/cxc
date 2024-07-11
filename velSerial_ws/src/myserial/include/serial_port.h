#ifndef _SERIAL_H_
#define _SERIAL_H_
#include <ros/ros.h>
#include <ros/time.h>
#include <boost/asio.hpp>

#include "std_msgs/String.h"//use data struct of std_msgs/String  
#include "std_msgs/Float32.h" 

#include <vector>

#define     HEAD_H              0xAA
#define     HEAD_R_H            0xAB
#define     HEAD_L              0x01
#define     HW_ADDR             0x30

#define     Board_Type          0x0101


#define     UART_GPIO_CONTROL   0x01000001
 
void serialInit();
void writeSpeed(uint32_t cmd, uint8_t *sData, uint16_t dataLen);
bool readSpeed(uint8_t *rData);
uint16_t Cal_CRC(const uint8_t *ptr, uint32_t length);
bool Check_CRC(uint8_t data_h, uint8_t data_l, uint8_t *getBuf, uint16_t len);
void Uart_Close();
 

#endif
