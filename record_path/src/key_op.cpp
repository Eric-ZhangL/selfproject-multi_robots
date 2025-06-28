	#include <ros/ros.h>
	#include <std_msgs/Char.h>
	#include <std_msgs/Int32.h>
	#include <stdio.h>
	#include <termios.h>
	#include <fcntl.h>
	#include <geometry_msgs/Twist.h>

	int kbhit(void)
	{
	    struct termios oldt, newt;
	    int ch;
	    int oldf;
	    tcgetattr(STDIN_FILENO, &oldt);
	    newt = oldt;
	    newt.c_lflag &= ~(ICANON | ECHO);
	    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
	    ch = getchar();
	    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	    fcntl(STDIN_FILENO, F_SETFL, oldf);
	    if(ch != EOF)
	    {
		ungetc(ch, stdin);
		return 1;
	    }
	    return 0;
	}

	int getch()
	{
	    struct termios oldt,newt;
	    int ch;
	    tcgetattr( STDIN_FILENO, &oldt );
	    newt = oldt;
	    newt.c_lflag &= ~( ICANON | ECHO );
	    tcsetattr( STDIN_FILENO, TCSANOW, &newt );
	    ch = getchar();
	    tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
	    return ch;
	}


	int main(int argc,char **argv)
	{
	    
	    ros::init(argc, argv, "key_op");                              //解析参数，命名节点
	    ros::NodeHandle nh;                                               //创建句柄，实例化node
	    ros::Publisher pub1 = nh.advertise<geometry_msgs::Twist>("/car_cmd", 1);//创建publisher
		
		ros::Publisher pub2 = nh.advertise<std_msgs::Int32>("key_record", 1);//创建publisher


	    ros::Rate loop_rate(100);         //循环频率
		std_msgs::Int32 msg;
		geometry_msgs::Twist mctr;
		while(ros::ok())
	    {
            //pub.publish(msg);
			
			if(kbhit())
			{
		        char c=getch();

				if(c=='r')  
				{
					msg.data=1;
					pub2.publish(msg);
					//printf("wsk\n");
				}
				else if(c!='w' && c!='a' && c!='s' && c!='d')
				{
					std_msgs::Int32 msg;
					msg.data=c;
					pub2.publish(msg);
				}
				
				
				// mctr.linear.x=0;
				mctr.angular.z=0;
                int stepl=1;
				int stepa=1;
				bool flag=false;
				if(c=='w') 
				 	mctr.linear.x+=stepl;
				else if(c=='s')  
					mctr.linear.x-=stepl; 
				else if(c=='a')  mctr.angular.z+=stepa;
				else if(c=='d')  mctr.angular.z-=stepa;

				if(mctr.linear.x>=1) mctr.linear.x=1;
				if(mctr.linear.x<=-1) mctr.linear.x=-1;
				if(mctr.angular.z>=0.4) mctr.angular.z=0.4;
				if(mctr.angular.z<=-0.4) mctr.angular.z=-0.4;
				// else if(c=='1')  mctr.mode=1,  flag=true;  
                // else if(c=='2')  mctr.mode=-1,  flag=true;
				pub1.publish(mctr);
				//printf("%d \n", mctr.speed);
			}
		    loop_rate.sleep();
	    }
	    return 0;

	}