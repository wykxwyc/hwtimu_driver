#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
#include <error.h>
#include <ros/ros.h>
#include <math.h>
#include <iostream>
#include <csignal>

using namespace std;

#define GR 10
#define RR 200
#define GRAVITY 9.7936
#define PI 3.1415927
#define SERIAL_LOCATION "/dev/ttyS2"
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "imu_talker");
	ros::NodeHandle n;
	ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu/data_raw", 1000);

	int fd;  //文件句柄
	struct termios newtio,oldtio;
	bool issync=false;
	unsigned char buffer[256];

	double roll_angle,pitch_angle;  //度
	double roll_rate,pitch_rate,yaw_rate;  //度每秒
	double acc_x,acc_y,acc_z;  //G
	double tempture,time_us;  
	int status;

	/*初始化串口*/
	cout<<SERIAL_LOCATION<<endl;
	fd=open(SERIAL_LOCATION,O_RDWR|O_NOCTTY|O_NDELAY);
	if(fd==-1){
		ROS_INFO("can't open serial port");
		return -1;
	}
	if(fcntl(fd,F_SETFL,0)<0){
		ROS_INFO("fcntl failed");
		return -1;
	}
	if(tcgetattr(fd,&oldtio)!=0){
		ROS_INFO("can't get old termios");
		return -1;
	}
	bzero(&newtio,sizeof(newtio));
	//波特率38400,8比特数据，１个开始位，１个停止位，无奇偶性，no flow control
	newtio.c_cflag |= CLOCAL|CREAD;
	newtio.c_cflag &= ~CSIZE;
	newtio.c_cflag |= CS8; //8比特数据
	newtio.c_cflag |= ~PARENB;  //无奇偶性
	cfsetispeed(&newtio,B38400);  //波特率
	cfsetospeed(&newtio,B38400);
	newtio.c_cflag &= ~CSTOPB;  //1个停止位
	newtio.c_cflag &= ~CRTSCTS;
	newtio.c_cc[VTIME] = 0;
	newtio.c_cc[VMIN] = 25;
	tcflush(fd,TCIFLUSH);
	if(tcsetattr(fd,TCSANOW,&newtio)!=0){
		ROS_INFO("set TCSANOW error");
		return -1;
	}
	ROS_INFO("init serial success!");

	/*发送命令*/
	int sendnum;
	//polled mode
	sendnum = write(fd,"P",1);
	if(sendnum!=1){
		ROS_INFO("send P error");
		return -1;
	}
	//angle mode
	sendnum = write(fd,"a",1);
	if(sendnum!=1){
		ROS_INFO("send a error");
		return -1;
	}
	//vg800 packet format
	sendnum = write(fd,"E",1);
	if(sendnum!=1){
		ROS_INFO("send E error");
		return -1;
	}
	//continuous mode
	sendnum = write(fd,"C",1);
	if(sendnum!=1){
		ROS_INFO("send C error");
		return -1;
	}
	ROS_INFO("send command success");

	//read
	int readnum;
	while(issync!=true&&ros::ok()){
		memset(buffer,0,sizeof(buffer));
		readnum=read(fd,buffer,25);
		cout<<"readnum: "<<readnum<<endl;
		for(int i=0;i<25;i++){
			printf("%02x ",buffer[i]);
		}
		printf("\n");
		if(buffer[0]==0xFF&&buffer[24]==0xFF){
			ROS_INFO("Header check success!");
			unsigned int sum=0;
			for(int i=1;i<23;i++){
				sum+=buffer[i];
			}
			unsigned int remainder=sum&0xFF;
			printf("remainder: %02x\n",remainder);
			printf("buffer: %02x\n",buffer[23]);
			if(remainder==buffer[23]){
				issync=true;
			}else{
				continue;
			}
		}else{
			ROS_INFO("Header check fail!");
			continue;
		}
	}
	ROS_INFO("sync success!");

	int count=0;
	while (ros::ok()&&issync)
	{
		sensor_msgs::Imu imu_msg;
		/*接受命令并解析*/
		//FF header在最后
		readnum=read(fd,buffer,24);
		int sum=0;
		for(int i=0;i<22;i++){
			sum+=buffer[i];
		}
		int remainder=sum&0xFF;
		if(remainder!=buffer[22]){
			issync=false;
			ROS_INFO("remainder not equal checksum!");
		}
		if(buffer[23]!=0xFF){
			issync=false;
			ROS_INFO("FF Header error!");
		}
		double scale=pow(2,15);
		int MSB,LSB;
		int tmp;

		MSB=((int)buffer[0])<<8;
		LSB=(int)buffer[1];
		tmp=MSB+LSB;
		if((tmp&0x00008000)==0x00008000){
			tmp=tmp|0xFFFF0000;
		}
		roll_angle=tmp*180/scale;
			
		MSB=((int)buffer[2])<<8;
		LSB=(int)buffer[3];
		tmp=MSB+LSB;
		if((tmp&0x00008000)==0x00008000){
			tmp=tmp|0xFFFF0000;
		}
		pitch_angle=tmp*180/scale;

		MSB=((int)buffer[4])<<8;
		LSB=(int)buffer[5];
		tmp=MSB+LSB;
		if((tmp&0x00008000)==0x00008000){
			tmp=tmp|0xFFFF0000;
		}
		roll_rate=tmp*RR*1.5/scale;

		MSB=((int)buffer[6])<<8;
		LSB=(int)buffer[7];
		tmp=MSB+LSB;
		if((tmp&0x00008000)==0x00008000){
			tmp=tmp|0xFFFF0000;
		}
		pitch_rate=tmp*RR*1.5/scale;

		MSB=((int)buffer[8])<<8;
		LSB=(int)buffer[9];
		tmp=MSB+LSB;
		if((tmp&0x00008000)==0x00008000){
			tmp=tmp|0xFFFF0000;
		}
		yaw_rate=tmp*RR*1.5/scale;

		MSB=((int)buffer[10])<<8;
		LSB=(int)buffer[11];
		tmp=MSB+LSB;
		if((tmp&0x00008000)==0x00008000){
			tmp=tmp|0xFFFF0000;
		}
		acc_x=tmp*GR*1.5/scale;

		MSB=((int)buffer[12])<<8;
		LSB=(int)buffer[13];
		tmp=MSB+LSB;
		if((tmp&0x00008000)==0x00008000){
			tmp=tmp|0xFFFF0000;
		}
		acc_y=tmp*GR*1.5/scale;

		MSB=((int)buffer[14])<<8;
		LSB=(int)buffer[15];
		tmp=MSB+LSB;
		if((tmp&0x00008000)==0x00008000){
			tmp=tmp|0xFFFF0000;
		}
		acc_z=tmp*GR*1.5/scale;

		MSB=((int)buffer[16])<<8;
		LSB=(int)buffer[17];
		tmp=MSB+LSB;
		tempture=(tmp-32767)/327.68;

		MSB=((int)buffer[18])<<8;
		LSB=(int)buffer[19];
		tmp=MSB+LSB;
		time_us=tmp;

		MSB=((int)buffer[20])<<8;
		LSB=(int)buffer[21];
		tmp=MSB+LSB;
		status=tmp;
			
		imu_msg.header.seq=count;
		imu_msg.header.stamp=ros::Time::now();
		imu_msg.header.frame_id="imu_link";

		imu_msg.orientation.x=0;
		imu_msg.orientation.y=0;
		imu_msg.orientation.z=0;
		imu_msg.orientation.w=0;
			
		//x,y,z的对应关系要搞清楚
		imu_msg.angular_velocity.x=roll_rate/180*PI;
		imu_msg.angular_velocity.y=pitch_rate/180*PI;
		imu_msg.angular_velocity.z=yaw_rate/180*PI;

		imu_msg.linear_acceleration.x=acc_x*GRAVITY;
		imu_msg.linear_acceleration.y=acc_y*GRAVITY;
		imu_msg.linear_acceleration.z=acc_z*GRAVITY;

		ROS_INFO("count:%d",count);
		imu_pub.publish(imu_msg);
		count++;
	}

	return 0;
}
