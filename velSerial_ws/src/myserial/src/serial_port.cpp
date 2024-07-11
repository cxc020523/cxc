#include "serial_port.h"
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/basic_io_object.hpp>
 
using namespace std;
using namespace boost::asio;

boost::asio::io_service iosev;
boost::asio::serial_port sp(iosev, "/dev/ttyUSB0");
boost::system::error_code err;

const uint16_t crc_table[16]=
{
  //CRC-16
  0x0000,0x1021,0x2042,0x3063,0x4084,0x50A5,0x60C6,0x70E7,
  0x8108,0x9129,0xA14A,0xB16B,0xC18C,0xD1AD,0xE1CE,0xF1EF
};

uint16_t Cal_CRC(const uint8_t *ptr, uint32_t length)
{
  static uint16_t crc;
  static uint8_t dat;

  crc=0xffff;
  if(length > 0)
  {
    while(length--)
    {
      dat=((uint8_t)(crc >> 12));
      crc <<= 4;
      crc ^= crc_table[dat ^ ((*ptr) >> 4)];

      dat=((uint8_t)(crc >> 12));
      crc <<= 4;
      crc ^= crc_table[dat ^ (*ptr & 0x0f)];
      ptr++;
    }
  }
  return crc;
}

bool Check_CRC(uint8_t data_h, uint8_t data_l, uint8_t *getBuf, uint16_t len)
{
  uint16_t checkCrc;
  checkCrc = (uint16_t)data_h << 8;
  checkCrc += data_l;
  uint16_t getCrc = Cal_CRC(getBuf, len);

  if(getCrc == checkCrc)
  {
    ROS_INFO("get checkdata %4x correctly!\n", getCrc);
    return true;
  }
  else  
    return false;
}


void serialInit()
{
  sp.set_option(serial_port::baud_rate(115200));
  sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
  sp.set_option(serial_port::parity(serial_port::parity::none));
  sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
  sp.set_option(serial_port::character_size(8));    
}
void Uart_Close()
{
  //sp.cancel();
  //ROS_INFO("cancel ok!");

  sp.close();
  //ROS_INFO("close ok!");
}
 

void writeSpeed(uint32_t cmd, uint8_t *sData, uint16_t dataLen)
{
  uint8_t vUartData[512];
  uint16_t vUartLen = dataLen + 0x0D;

  vUartData[0] = HEAD_H;
  vUartData[1] = HEAD_L;
  vUartData[2] = (uint8_t)((vUartLen >> 8) & 0x00FF);
  vUartData[3] = (uint8_t)(vUartLen & 0x00FF);
  vUartData[4] = HW_ADDR;
  vUartData[5] = (uint8_t)((Board_Type >> 8) & 0x00FF);
  vUartData[6] = (uint8_t)(Board_Type & 0x00FF);
  vUartData[7] = (uint8_t)((cmd >> 24) & 0x000000FF);
  vUartData[8] = (uint8_t)((cmd >> 16) & 0x000000FF);
  vUartData[9] = (uint8_t)((cmd >> 8) & 0x000000FF);
  vUartData[10] = (uint8_t)(cmd & 0x000000FF);

  memcpy(vUartData+11, sData, dataLen);

  vUartData[vUartLen-2] = (uint8_t)((Cal_CRC(vUartData, vUartLen-2)>>8)&0x00FF);
  vUartData[vUartLen-1] = (uint8_t)(Cal_CRC(vUartData, vUartLen-2)&0x00FF);
  
  boost::asio::write(sp, boost::asio::buffer(vUartData));
  ROS_INFO("send data over!");
}


bool readSpeed(uint8_t *rData)
{
  char i, length = 0;
  uint16_t checkCRC, getCRC;
  uint8_t buf[128]={0};
  
  ROS_INFO("start to read data!");
  boost::array<uint8_t, 15> receive_array;
  try
  {     
    boost::asio::read(sp, boost::asio::buffer(receive_array));
  }  
  catch(boost::system::system_error &err)
  {
    ROS_INFO("read_until error");
  }

  for(int i = 0; i < receive_array.size(); i++)
  {
    buf[i] = receive_array[i];
  }

  length = receive_array.size();
  //ROS_INFO("check start!\n");
  //
  if(receive_array[0] != HEAD_R_H || receive_array[1] != HEAD_L)
  {
    ROS_ERROR("Received message header error!");
    return false;
  }

  if(Check_CRC(buf[length - 2], buf[length - 1], buf, length))
  {
    ROS_ERROR("Received data check sum error!");
    return false;
  }

  ROS_INFO("get data %2x%2x \n", buf[11], buf[12]);

  return true;
}

