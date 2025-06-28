#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <vector>
#include <string>
#include <std_msgs/Float32MultiArray.h>
#include "Public.h"
#include <std_msgs/String.h>
#include "record_path/motor_state.h"
#include "record_path/motor_ctr.h"
#include "record_path/gps_data.h"

using namespace std;

#define RemoteIP "192.168.3.103"

const float Pi=3.14159;

int  sock;
char UDP_recvbuf[1024] = {0};
int  UDP_recvlen=0;

ros::Publisher motor_pub;
ros::Publisher gps_pub;
ros::Subscriber motor_sub;

#define ERR_EXIT(m) \
    do { \
    perror(m); \
    exit(EXIT_FAILURE); \
    } while (0)


        
//解析数据包：从数据包中读出各部分信息
void GetUGVState(char *recbuf)
{
   vector<string> strs,substrs;
   
   strs=split(recbuf,"\n");
   //printf("%s\n",recbuf);
   for(int i=0;i<strs.size();i++)
   {
       substrs=split(strs.at(i)," ");
       if(substrs.size()>=32 && substrs.at(0)=="$Motor")  
       {
            record_path::motor_state msg;

            msg.mode=atoi(substrs.at(1).c_str());
            msg.speed=atoi(substrs.at(2).c_str());
            msg.angle=atoi(substrs.at(3).c_str());
            for(int j=0;j<4;j++)
            {
                msg.enabled[j]=atoi(substrs.at(4+7*j).c_str());
                msg.WNvalue[j]=atoi(substrs.at(5+7*j).c_str());
                msg.DIvalue[j]=atoi(substrs.at(6+7*j).c_str());
                msg.DOvalue[j]=atoi(substrs.at(7+7*j).c_str());
                msg.bv[j]=atof(substrs.at(8+7*j).c_str());
                msg.iq[j]=atof(substrs.at(9+7*j).c_str());
                msg.spd[j]=atof(substrs.at(10+7*j).c_str());
            }

            motor_pub.publish(msg);
       }
       else if(substrs.size()>=9 && substrs.at(0)=="$GPS" && substrs.at(1)=="B")  
       {
            record_path::gps_data msg;
            msg.quality = atoi(substrs.at(2).c_str());
            msg.lat = atof(substrs.at(3).c_str());
            msg.lon = atof(substrs.at(4).c_str());
            msg.utm_x = atof(substrs.at(5).c_str());
            msg.utm_y = atof(substrs.at(6).c_str());
            msg.angle2north = atof(substrs.at(7).c_str());
            msg.speed = atof(substrs.at(8).c_str());
            gps_pub.publish(msg);
          
            //printf("quality=%d %.2f %.2f  \n ",msg.quality, x, y);
       }
   }
}

/*线程：通过UDP读取底层数据*/
void *echo_ser(void *arg)
{
    int len;
    char recvbuf[1024] = {0};
    struct sockaddr_in peeraddr;
    socklen_t peerlen;
    while(1)
    {
        peerlen = sizeof(peeraddr);
        memset(recvbuf, 0, sizeof(recvbuf));
        len = recvfrom(sock, recvbuf, sizeof(recvbuf), 0, (struct sockaddr *)&peeraddr, &peerlen);
        string ip=inet_ntoa(peeraddr.sin_addr);   

        if(ip==RemoteIP)  GetUGVState(recvbuf);
        if (len <= 0)
        {
            if (errno == EINTR)
                continue;
            ERR_EXIT("recvfrom error");
        }
    }
    close(sock);
    pthread_exit(NULL);
}


void UDP_Init(int port)
{
    if ((sock = socket(PF_INET, SOCK_DGRAM, 0)) < 0)   ERR_EXIT("socket error");

 
    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(port);
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);

    printf("监听%d端口\n",port);
    if (bind(sock, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
        ERR_EXIT("bind error");

    pthread_t thread;
    int res=pthread_create(&thread,NULL,echo_ser,NULL);//创建一个线程，用于发布话题
    if(res!=0)
    {
        printf("Create thread failed\n");
        exit(res);
    }
}

void Close_UDP(void)
{
    close(sock);
}

void UDP_Send(char *ip, int port, char *str)
{
    struct sockaddr_in sendaddr;
	
    sendaddr.sin_family = AF_INET;
    sendaddr.sin_port = htons(port);
    sendaddr.sin_addr.s_addr = inet_addr(ip);
	sendto(sock, str, strlen(str), 0, (struct sockaddr *)&sendaddr, sizeof(struct sockaddr_in));
}

void MyCallback(const record_path::motor_ctr::ConstPtr& msg)
{
    char buf[1000];
    sprintf(buf, "$Motor B %d %d %d", msg->mode, msg->speed, msg->angle);
    
    
    UDP_Send(RemoteIP, 8080, buf);
    //printf("%s\n",buf);
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "MyUDP");  //初始化节点
	ros::NodeHandle n;
   
    motor_pub = n.advertise<record_path::motor_state>("Motor_State",10);
    gps_pub = n.advertise<record_path::gps_data>("GPS_Data",10);
    motor_sub = n.subscribe("Motor_Ctr", 200, MyCallback);
    // ros::Publisher motorctr_pub = n.advertise<ugv_bit::motor_ctr>("Motor_Ctr", 1);

    ros::Rate loop_rate(100);   //定义程序循环的频率
	UDP_Init(8090);  //初始化UDP
    int count;
	while (ros::ok())
	{
        count++;
        
        record_path::gps_data msg;
        msg.quality = 4;
        msg.utm_x = 440572.39+count*0.005;
        msg.utm_y = 4423926.44+count*0.0006;
        msg.angle2north = -90;
        gps_pub.publish(msg);


        ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
