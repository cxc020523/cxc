#include "ros/ros.h"
#include "std_msgs/String.h"
#include "serial_port.h"
 
using namespace std;


int main(int agrc,char **argv)
{
    ros::init(agrc,argv,"vel");
    ros::NodeHandle nh;
    // 设置执行频率50hz
    ros::Rate loop_rate(100);

    
    // 串口初始化，也可以使用Rosserial实现
    serialInit();
    uint8_t sData[128] = {0};
    uint8_t rData[128] = {0};
    int func(0);
    ROS_INFO("we start\n");
    while(ros::ok())
    {
        cout << "Your function number is: ";
        cin >> func;

        if(func == 5)
        {
            sData[0] = 0x02;
            writeSpeed(UART_GPIO_CONTROL, sData, 1);
            readSpeed(rData);
            
            //loop_rate.sleep();
            ros::shutdown();
            loop_rate.sleep(); 
            Uart_Close();
            break;
        }
        else
        {
            switch (func)
            {                    
                case 1:    
                    sData[0] = 0x01;
                    sData[1] = 0x00;        
                    break;
                case 2:     
                    sData[0] = 0x00;
                    sData[1] = 0x01;        
                    break;

                case 3:     
                    sData[0] = 0x01;
                    sData[1] = 0x01;        
                    break;
                case 4:     
                    sData[0] = 0x00;
                    sData[1] = 0x00;        
                    break;
                default:    
                    ROS_ERROR_STREAM("No this function number!!!");     
                    break;
            }
        }
        

        ros::spinOnce();
        // 发送命令给STM32
	    writeSpeed(UART_GPIO_CONTROL, sData, 2);
        
	    readSpeed(rData);
        
        loop_rate.sleep();
    }
    ROS_INFO("project is about to close\n");
    return 0;
}

