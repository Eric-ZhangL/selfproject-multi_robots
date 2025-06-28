#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h> 
#include <sensor_msgs/Imu.h>

using namespace std;

class IMU_ml7600
{
private:
    serial::Serial *ser;
    ros::NodeHandle *nh;
    ros::Publisher pub;

public:
    float angle[3];
    float angle_vel[3];
    float linear_acc[3];

    IMU_ml7600()
    {
        nh=new ros::NodeHandle("~");  //  局部空间

        string devname="";
        int baud=0;
        nh->getParam("devname",devname);
        nh->getParam("baud",baud);

        if(devname!="" && baud>0) 
        {
            ser=new serial::Serial;
            ser->setPort(devname); 
            printf("%s\n",devname.c_str());
            ser->setBaudrate(baud); 
            serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
            ser->setTimeout(to); 
            ser->open();
        }
        else ser=NULL;     

        //SetZero();
        // printf("OK\n");
        pub = nh->advertise<sensor_msgs::Imu>("/imu_data", 1);
    }

    void SendCmd()
    {
        if(ser==NULL) return;  
        unsigned char buf[100]={01, 03, 00, 01, 00, 0x12, 0x94, 0x07};
        ser->write(buf,8);
    }

    void SetZero()
    {
        if(ser==NULL) return;
        unsigned char buf[100]={01, 06, 00, 0x13, 00, 01, 0xb9, 0xcf};
        ser->write(buf,8);
    }

    int GetData()
    {
        int flag=0;
        if(ser==NULL) return flag;

        int n=ser->available();
        if(n>=41) 
        {
            //ROS_INFO("%d",ser.available());
            unsigned char buf[100];
            n=ser->read(buf,n);
            ser->flushInput();
            if(n==41 && buf[0]==0x01 && buf[1]==0x03 && buf[2]==0x24)
            {
                flag=1;
                for(int i=0;i<3;i++)
                {
                    unsigned char tmbuf[4]={buf[4*i+6],buf[4*i+5],buf[4*i+4],buf[4*i+3]};
                    memcpy(&angle[i],tmbuf,4);
                }
                for(int i=0;i<3;i++)
                {
                    unsigned char tmbuf[4]={buf[4*i+6+12],buf[4*i+5+12],buf[4*i+4+12],buf[4*i+3+12]};
                    memcpy(&linear_acc[i],tmbuf,4);
                }
                for(int i=0;i<3;i++)
                {
                    unsigned char tmbuf[4]={buf[4*i+6+24],buf[4*i+5+24],buf[4*i+4+24],buf[4*i+3+24]};
                    memcpy(&angle_vel[i],tmbuf,4);
                }

                sensor_msgs::Imu msg;
                msg.header.stamp = ros::Time::now();
                msg.header.frame_id = "base_link";
                msg.orientation.x=angle[0];
                msg.orientation.y=angle[1];
                msg.orientation.z=angle[2];
                msg.angular_velocity.x=angle_vel[0];
                msg.angular_velocity.y=angle_vel[1];
                msg.angular_velocity.z=angle_vel[2];
                msg.linear_acceleration.x=linear_acc[0];
                msg.linear_acceleration.y=linear_acc[1];
                msg.linear_acceleration.z=linear_acc[2];
                pub.publish(msg);
            }
        }
        return flag;       
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_ml7600");
    IMU_ml7600 imu;
    
    ros::Rate rate(80);
    while (true)
    {
        imu.SendCmd();
        usleep(8000);  // 9ms
        //rate.sleep();
        imu.GetData();
        //printf("%d \n",);    
    }
       
    return 0;
}