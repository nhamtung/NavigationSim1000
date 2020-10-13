#include <vector>
#include <string>
#include <ros/ros.h>
#include <sys/stat.h>
#include "mbrtu/modbusrtu.h"
#include <geometry_msgs/Twist.h>
#include <boost/thread/thread.hpp>
#include <diagnostic_msgs/KeyValue.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <driver_blvd_controller/speed_wheel.h>


/* Globle value */
int speed[2];
uint16_t alarm_status[2], feedback_speed[2], warning_status[2];
char port[30];    //port name
int baud;     	  //baud rate 
int check_connect;
struct stat sb;
clock_t begin;

//Process ROS receive from navigation message, send to uController
void navigationCallback(const driver_blvd_controller::speed_wheel& robot)
{
	speed[0] = robot.wheel_letf;
    speed[1] = robot.wheel_right;

    if(check_connect == 0)
    {
    	ROS_INFO("Connected ");
	    for(int i= 0; i<2; i++) ROS_INFO("  ");
	    ROS_INFO("                 Wheel left      Wheel right");
	    ROS_INFO("Command speed :  %d              %d", speed[0], speed[1]);
		ROS_INFO("Feedback speed:  %d              %d",(int16_t)feedback_speed[0],(int16_t)feedback_speed[1]);
		ROS_INFO("Warning record:  %x              %x",warning_status[0],warning_status[1]);
		ROS_INFO("Alarm record  :  %x              %x",alarm_status[0],alarm_status[1]);
    } 
} //navigationCallback

void infomationLeftDriver(int* publish_rate)
{ 

	ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();
	/* Publisher */
	ros::Publisher diagnostic_pub = nh->advertise<diagnostic_msgs::DiagnosticArray>("Diagnotics_LeftDriver", 10);
	ros::Rate loop_rate(*publish_rate);
	
	diagnostic_msgs::DiagnosticArray dia_array;

    std::vector<diagnostic_msgs::DiagnosticStatus> Driver;
    std::vector<diagnostic_msgs::KeyValue> feedbackSpeed;
    std::vector<diagnostic_msgs::KeyValue> alarmRecord;
    std::vector<diagnostic_msgs::KeyValue> warningRecord;

	while (ros::ok())
	{
		for(int n = 0; n < 2; n++)
		{
			Driver[n].name = "Oriental BLVD20KM";
			Driver[n].hardware_id = "0x01";
			if (check_connect != 0)
			{
				Driver[n].level = diagnostic_msgs::DiagnosticStatus::ERROR;
				Driver[n].message = "Driver disconnected. Check connection,plaese!!";
			}else if(alarm_status[n-1] != 0)
				{
					Driver[n].level = diagnostic_msgs::DiagnosticStatus::ERROR;
					Driver[n].message = "Driver is alarm. Reset device, please!!";
				}
			else if (warning_status[n-1] !=0)
				{
					Driver[n].level = diagnostic_msgs::DiagnosticStatus::WARN;
					Driver[n].message = " Warning from driver. Attention!!";
				}
			else
				{
					Driver[n].level = diagnostic_msgs::DiagnosticStatus::OK;
					Driver[n].message = "Driver seem to be ok.";
				}
			feedbackSpeed[n].key = "Feed back speed";
			feedbackSpeed[n].value = std::to_string(feedback_speed[n]);
			Driver[n].values.push_back(feedbackSpeed[n]);

			warningRecord[n].key = "Warning record";
			warningRecord[n].value = std::to_string(warning_status[n]);
			Driver[n].values.push_back(warningRecord[n]);

			alarmRecord[n].key = "Alarm record";
			alarmRecord[n].value = std::to_string(alarm_status[n]);
			Driver[n].values.push_back(alarmRecord[n]);
			
			dia_array.status.push_back(Driver[n]);
		}

    	diagnostic_pub.publish(dia_array);

		loop_rate.sleep();
	}

}//infomationLeftDriver

int main(int argc, char **argv)
{
  	baud = DEFAULT_BAUDRATE;
	if (argc > 1) {
		if(sscanf(argv[1],"%d", &baud)!=1) {
		  ROS_ERROR("ucontroller baud rate parameter invalid");
		  return 1;
		}
	}

	strcpy(port, DEFAULT_SERIALPORT);
	if (argc > 2)
	strcpy(port, argv[2]);

	/* spawn another thread */
	// int rate_b = 20; // 20 Hz
	// boost::thread thread_one(infomationLeftDriver, &rate_b);
 
	/*create ros node*/
	ros::init(argc, argv, "Driver_motor");
	ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();
	/* Subscriber */
    ros::Subscriber navigation =  nh->subscribe("cmd_vel_to_wheel", 10,navigationCallback); 
    /* Publisher */
    //driver_blvd_controller::speed_wheel encoder;
    ros::Publisher speed_wheel = nh->advertise<driver_blvd_controller::speed_wheel>("wheel_encoder", 10);
	
	ros::Rate loop_rate(10); 
	while(ros::ok())
	{
		/* onpen comport */
		ROS_INFO("connection initializing (%s) at %d baud", port, baud);
		Mb_open_device(port,baud,1,8,1); /*even , 8 bit, 1 stop_bit*/
		
		if(stat(DEFAULT_SERIALPORT, &sb) == 0)
		{
			for (uint8_t i = 1; i < 3; i++)
			{	
				writeResetAlarm(i); 
				writeSpeedControlMode(i,BLVD02KM_SPEED_MODE_USE_DIGITALS);
				writeAcceleration(i,5);
				writeDeceleration(i,2);
				writeSpeed(i,BLVD20KM_SPEED_MIN);
				writeStop(i);
				clearAlarmRecords(i); 
				clearWarningRecords(i);
			}
		}
		sleep(1);
		while(ros::ok())
		{
			if((clock() - begin)/CLOCKS_PER_SEC >= 1)
			{
				ROS_WARN("156-check_connect: %f", check_connect);
				check_connect = stat(DEFAULT_SERIALPORT, &sb);
				begin = clock();
			} 
				
			if(check_connect != 0){
				ROS_WARN("162-check_connect: %f", check_connect);
				for(int i= 0; i<4; i++) ROS_INFO("  ");
				ROS_INFO("Disconnected");
				break;
			} 

			for(uint8_t i = 1; i < 3; i++)
			{
				if(speed[i-1] > 0){
					writeForward(i);
				}else if(speed[i-1] < 0){
					writeReverse(i); 
				}else writeStop(i);
				writeSpeed(i,abs(speed[i-1]));
				feedbackSpeed(i,&feedback_speed[i-1]);
				readAlarm(i,&alarm_status[i-1]);
				readWarning(i,&warning_status[i-1]);
			}

			// encoder.wheel_letf = (double)(feedback_speed[0]/30);
			// encoder.wheel_right = (double)(feedback_speed[1]/30);
			//speed_wheel.publish(encoder);
			loop_rate.sleep();
			ros::spinOnce();
		}

		Mb_close_device();
		sleep(2);
	}
	//thread_one.join();

	return 0;
}

