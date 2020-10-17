#include "mbtcp/modbus.h"
#include "fx5u_hardware/fx5u_hardware.h"
#include <diagnostic_msgs/KeyValue.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <ros/ros.h>
using namespace std;

#define NO_CL  0
#define RED    1
#define GREEN  2
#define YELLOW 3
#define BULE   4
#define PINK   5
#define OCEAN  6
#define WHITE  7


void drivermotor_cmdcallback(const diagnostic_msgs::DiagnosticStatus& status)
{

}

void mlse_cmdcallback(const diagnostic_msgs::DiagnosticStatus& status)
{

}

int main(int argc, char **argv)
{
    /* create a modbus object */
    modbus *fx5uc = new modbus("192.168.1.51", 502);
    /* set slave id */
    fx5uc->modbus_set_slave_id(1);
    /* connect with the server */
    fx5uc->modbus_connect();  
    /* create hardware */
    FX5U_series device;
    /* create ros ndoe */
    ros::init(argc, argv, "PLC_control");
    ros::NodeHandle nh;
    ros::Rate loop_rate(20);
    /* Publisher */
    ros::Publisher cmd_PLC;
     cmd_PLC = nh.advertise<diagnostic_msgs::DiagnosticStatus>("PLC_infomation", 20);

    diagnostic_msgs::DiagnosticArray dir_array;
	diagnostic_msgs::DiagnosticStatus PLC;
    diagnostic_msgs::KeyValue LED;
    diagnostic_msgs::KeyValue Dock;
    diagnostic_msgs::KeyValue Xilanh;

    bool bitM_echo[10];
    bool m1m2m3[3] = {OFF,OFF,OFF};
    while(ros::ok())
    {   
        fx5uc->modbus_read_coils(Mbit, 10,bitM_echo); 
        //ROS_INFO("M0 = %d M5 = %d M6 = %d M7 = %d",bitM[0],bitM[5],bitM[6],bitM[7]);  
        if(bitM_echo[0] == ON)// Nếu M0 on 
        {
            if(bitM_echo[5] == ON){
                device.D[1] = RED;
                m1m2m3[0] = ON;m1m2m3[1] = ON;m1m2m3[2] = OFF;
            }

            else if(bitM_echo[6] == ON){
                device.D[1] = YELLOW;
                m1m2m3[0] = ON;m1m2m3[1] = ON;m1m2m3[2] = OFF;
            }
            else if(bitM_echo[7]== ON){
                device.D[1] = YELLOW;
                m1m2m3[0] = ON;m1m2m3[1] = OFF;m1m2m3[2] = OFF;
            }
            else if(bitM_echo[8]== ON){
            device.D[1] = GREEN;
            m1m2m3[0] = ON;m1m2m3[1] = OFF;m1m2m3[2] = OFF;
            }
            else if(bitM_echo[9]== ON){
            device.D[1] = OCEAN;
            m1m2m3[0] = ON;m1m2m3[1] = OFF;m1m2m3[2] = OFF;
            }
            fx5uc->modbus_write_coils(Mbit+1, 3,m1m2m3); 
            fx5uc->modbus_write_register(0, device.D[1]); 
        } else ROS_INFO("fx5uc_controller.cpp-80-not listen"); 


        loop_rate.sleep();
        ros::spinOnce();
    }
    //close connection and free the memory
    fx5uc->modbus_close();  
    delete(fx5uc);
    return 0;
}  