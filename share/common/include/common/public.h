#ifndef ABC_ABC_H
#define ABC_ABC_H

#include <vector>
#include <string>
#include <sys/time.h>
#include <unistd.h>
#include <sys/io.h>
#include <memory.h>
#include <dirent.h>
#include <math.h>

#include <stdio.h>
#include <fcntl.h>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>



#define Rad2Deg 180/3.1415926
#define Deg2Rad 3.1415926/180
#define pi 3.1415926


//#define rad2deg 57.295780490442
const float rad2deg = 57.295780490442;
using namespace std;

struct _Point
{
    double x,y;
};

struct _Spcs
{
    float r, a;
};

struct Position_Err  // 航向误差， 横向误差，纵向误差
{
    float dx, dy;
    float angle;
    float ds;
};

/**
 * @description:将当前角度转换到0～2pi 
 * @param {type} 
 * @return: 
 */
inline double cast_from_0_to_2PI_Angle(const double& ang)
{
    double angle = 0;
    if (ang < -2.0 * M_PI || ang > 2.0 * M_PI) {
        angle = fmod(ang, 2.0 * M_PI);
    } else
        angle = ang;

    if (angle < 0) {
        angle = 2.0 * M_PI + angle;
    }
    return angle;
}

_Spcs P2P(_Point p1,_Point p2)
{
    _Spcs p;
    double x=p2.x-p1.x, y=p2.y-p1.y;

    p.r=x*x+y*y;
    if(fabs(p.r)<0.000001) p.r=0;
    else p.r=sqrt(p.r);

    if (fabs(y) < 0.0001)
    {
        if (x >= 0) p.a = pi / 2;
        else p.a = -pi / 2;
    }
    else if (y > 0) p.a = atan(x / y);
    //else if (x >= 0) p.a = pi - atan(x / y);
    else p.a = -pi + atan(x / y);     //  计算误差角度

    return p;
}

double P2P_r(_Point p1,_Point p2)
{
    _Spcs p=P2P(p1,p2);
    return p.r;
}

_Spcs P2P(_Point p)
{
    _Point p0={0,0};
    return P2P(p0,p);
}

double P2P(double x1,double y1,double x2,double y2)
{
    double r=(x1-x2)*(x1-x2)+(y1-y2)*(y1-y2);
    if(r>0.01) r=sqrt(r);
    else r=0;

    return r;
}

float P2P(geometry_msgs::Point p1,geometry_msgs::Point p2)
{
    float r;
    r=(p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y)+(p1.z-p2.z)*(p1.z-p2.z);
    r=sqrt(r);
    return r;
}

bool InPP(double p1,double p2,double px)
{
    bool r=false;
    if(px>=p1 && px<=p2) r=true;
    else if(px<=p1 && px>=p2) r=true;
    return r;
}



bool CheckPoint(_Point p)
{
//    if(fabs(p.x)+fabs(p.y)>10) return true;
//    return false;
    return fabs(p.x)+fabs(p.y)>10;
}

struct LS_Coeff     //最小二乘法二次多项式的三个系数
{
    double a0,a1,a2;
};

vector<string> getFileList(char *basePath,char *sep);
void GetPackagePath(char *packname,char *path);
vector<string> split(const string& s, const string& sep);
LS_Coeff Least_Square(vector<geometry_msgs::Point32> point);

class TTimer
{
private:
    struct timeval tv1, tv2;
    struct timezone tz;
    float t_off;
public:
    float value;

    TTimer()
    {
        Clear();
        value=0;
    }

    void Clear(float t=0)
    {
        gettimeofday(&tv1, &tz);
        tv2=tv1;
        t_off=t;
    }

    double GetValue(void)
    {
        double ret;
        gettimeofday(&tv2, &tz);
        ret = (tv2.tv_sec - tv1.tv_sec) + (tv2.tv_usec - tv1.tv_usec)* 0.000001+t_off;
        value=ret;
        return ret;
    }
};

class TDataFilter
{
private:
    double buf[200];
    int filternum;
    vector<double> temp;
    double mid_value;
public:
    double value;

    TDataFilter()
    {
        filternum=20;
        for(int i=0;i<200;i++) buf[i]=0;
    }

    TDataFilter(int n)
    {
        TDataFilter();
        filternum=n;
    }

    double GetMidValue(double v)
    {
        int i;
        if(buf[0]==0)
        {
           for(i=0;i<filternum;i++)
           {
               buf[i]=v;
           }
        }
        for(i=0;i<filternum-1;i++) buf[i]=buf[i+1];
        buf[filternum-1]=v;
         
        temp.clear();
        mid_value = 0;
        
        for(i=0;i<filternum;i++) 
        {
            temp.push_back(buf[i]);           
        }
        sort(temp.begin(),temp.end());
        
        mid_value=temp[filternum/2];
        return mid_value;
    }

    double GetValue(double v)
    {
        int i;
        for(i=0;i<filternum-1;i++) buf[i]=buf[i+1];
        buf[filternum-1]=v;
        value=0;
        for(i=0;i<filternum;i++) value=value+buf[i];
        value=value/filternum;
        return value;
    }
    void SetValue(double v)
    {
        for(int i=0;i<200;i++) buf[i]=v;  //将这200个数据初始化为全0
    }
};


class Thread
{
private:
    //当前线程的线程ID
    pthread_t tid;
    //线程的状态
    int threadStatus;
    //获取执行方法的指针
    static void * thread_proxy_func(void * args)
    {
        Thread * pThread = static_cast<Thread *>(args);
        pThread->run();
        return NULL;
    }

    //内部执行方法
    void* run1()
    {
        threadStatus = THREAD_STATUS_RUNNING;
        tid = pthread_self();
        run();
        threadStatus = THREAD_STATUS_EXIT;
        tid = 0;
        pthread_exit(NULL);
    }

public:
    //线程的状态－新建
    static const int THREAD_STATUS_NEW = 0;
    //线程的状态－正在运行
    static const int THREAD_STATUS_RUNNING = 1;
    //线程的状态－运行结束
    static const int THREAD_STATUS_EXIT = -1;

    //构造函数
    Thread()
    {
        tid = 0;
        threadStatus = THREAD_STATUS_NEW;
    }

    //线程的运行实体
    virtual void run()=0;

    //开始执行线程
    bool start()
    {
        int iRet = 0;
        pthread_create(&tid, NULL, thread_proxy_func, this) == 0;
    }

    //获取线程ID
    pthread_t getThreadID()
    {
        return tid;
    }

    //获取线程状态
    int getState()
    {
        return threadStatus;
    }

    //等待线程直至退出
    void join()
    {
        if (tid > 0)
        {
            pthread_join(tid, NULL);
        }
    }

    //等待线程退出或者超时
    void join(unsigned long millisTime)
    {
        if (tid == 0)
        {
            return;
        }
        if (millisTime == 0)
        {
            join();
        }else
        {
            unsigned long k = 0;
            while (threadStatus != THREAD_STATUS_EXIT && k <= millisTime)
            {
                usleep(100);
                k++;
            }
        }
    }
};


class TPosInfoProc
{
private:
   vector<geometry_msgs::Point32> pos,vel;
   vector<float> stime;
   TTimer tmr;
public:
   geometry_msgs::Point32 avg_vel,pre_pos,pre_vel;
   int filtercount;

   TPosInfoProc()
   {
       filtercount=10;
	   Clear();
   }

   void Clear()
   {
       pos.clear();
       vel.clear();
       stime.clear();
       tmr.Clear();
   }

   int CheckData(float x,float y,float z)
   {
        int r=0;
        float t= tmr.GetValue();
        float maxv=10;
        if(stime.size()==0) r=1;
        else 
        {
            float dt=t-stime.back();
            float vx=(x-pos.back().x)/dt;
            float vy=(y-pos.back().y)/dt;
            float vz=(z-pos.back().z)/dt;
            if(fabs(vx)<maxv && fabs(vy)<maxv && fabs(vz)<maxv)  r=1;
        }

        return r;
   }

   void Prediction(float t1,float t2)  //  预测位移和速度，以当前时刻为基准
   {
      vector<geometry_msgs::Point32> px,py,pz,vx,vy,vz;
      
	  float curt=tmr.GetValue();
      for(int i=0;i<stime.size();i++)  
	 	  if(stime[i]>=curt-t1) 
          {
             geometry_msgs::Point32 p;
             p.x=stime[i];
             p.y=pos[i].x;  px.push_back(p); 
             p.y=pos[i].y;  py.push_back(p); 
             p.y=pos[i].z;  pz.push_back(p); 
             p.y=vel[i].x;  vx.push_back(p); 
             p.y=vel[i].y;  vy.push_back(p); 
             p.y=vel[i].z;  vz.push_back(p); 
          }
      if(px.size()>3)
      {
           LS_Coeff coeff=Least_Square(px);
           pre_pos.x=coeff.a0+coeff.a1*(curt+t2)+coeff.a2*(curt+t2)*(curt+t2);
           coeff=Least_Square(py);
           pre_pos.y=coeff.a0+coeff.a1*(curt+t2)+coeff.a2*(curt+t2)*(curt+t2);
           coeff=Least_Square(pz);
           pre_pos.z=coeff.a0+coeff.a1*(curt+t2)+coeff.a2*(curt+t2)*(curt+t2);
           coeff=Least_Square(vx);
           pre_vel.x=coeff.a0+coeff.a1*(curt+t2)+coeff.a2*(curt+t2)*(curt+t2);
           coeff=Least_Square(vy);
           pre_vel.y=coeff.a0+coeff.a1*(curt+t2)+coeff.a2*(curt+t2)*(curt+t2);
           coeff=Least_Square(vz);
           pre_vel.z=coeff.a0+coeff.a1*(curt+t2)+coeff.a2*(curt+t2)*(curt+t2);
      }
   }

   int AddData(float x,float y,float z)
   {
       if(CheckData(x,y,z)==0) return 0;

	   //printf("check ok!\n");
       int r=0;
       float t= tmr.GetValue();  
       float dt;
       geometry_msgs::Point32 p;
       p.x=p.y=p.z=0;

       if(stime.size()>0)
       {
          dt=t-stime.back();
          p.x=(x-pos.back().x)/dt;
          p.y=(y-pos.back().y)/dt;
          p.z=(z-pos.back().z)/dt;

		  //printf("%.1f %.1f\n", x, pos[pos.size()-1].x);
       }
       vel.push_back(p);
       p.x=x, p.y=y, p.z=z;
       pos.push_back(p);
       stime.push_back(t);

       if(stime.size()>1000)  
       {
          stime.erase(stime.begin());
          pos.erase(pos.begin());
          vel.erase(vel.begin());
       }

       int N=stime.size();
	   avg_vel.x=avg_vel.y=avg_vel.z=0;
	   if(N>filtercount)
	   {
	    	for(int i=0; i<filtercount; i++)
			{
				avg_vel.x+=vel[N-1-i].x;
				avg_vel.y+=vel[N-1-i].y;
				avg_vel.z+=vel[N-1-i].z;
			} 
			avg_vel.x/=filtercount;
			avg_vel.y/=filtercount;
			avg_vel.z/=filtercount;

            Prediction(0.5,0.5);
            //printf("t=%.3f pos=%.2f %.2f   vel=%.2f %.2f\n",t,pos.back().x,pre_pos.x,vel.back().x,pre_vel.x);
			
			r=1;
	   }	

	   return r;
   }
};


vector<string> readFileList(char *basePath);
int FindMinID(vector<float> datalist);

uint16_t CRC16(uint8_t *ptr, uint32_t len);

#endif //ABC_ABC_H

