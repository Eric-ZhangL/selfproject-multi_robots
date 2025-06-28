#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QFileDialog>
#include <geometry_msgs/Twist.h>
#include <QDebug>
#include <nav_msgs/Path.h>

#include "teleop_pad.h"

namespace rviz_teleop_commander
{

void GetPackagePath(char *packname, char *path)
    {
        char cmd[100];
        sprintf(cmd, "rospack find %s", packname);
        FILE *fp = popen(cmd, "r");

        fgets(path, 256, fp);
        pclose(fp);
        // printf("%s\n",path);
    }

// 构造函数，初始化变量
TeleopPanel::TeleopPanel( QWidget* parent )
  : rviz::Panel(parent)
  ,saveflag(false)
{
    // 创建一个输入topic命名的窗口
    QVBoxLayout* topic_layout = new QVBoxLayout;
    // topic_layout->addWidget( new QLabel( "Teleop Topic:" ));
    // output_topic_editor_ = new QLineEdit;
    // topic_layout->addWidget( output_topic_editor_ );
    // output_topic_editor_->setText("cmd_vel");

    // // 创建一个输入线速度的窗口
    // topic_layout->addWidget( new QLabel( "Linear Velocity:" ));
    // output_topic_editor_1 = new QLineEdit;
    // topic_layout->addWidget( output_topic_editor_1 );
    // output_topic_editor_1->setText("2");

    // // 创建一个输入角速度的窗口
    // topic_layout->addWidget( new QLabel( "Angular Velocity:" ));
    // output_topic_editor_2 = new QLineEdit;
    // topic_layout->addWidget( output_topic_editor_2 );
    // output_topic_editor_2->setText("0");

    // 创建按钮的窗口
    QHBoxLayout* button_layout1 = new QHBoxLayout;  //VH区别 H是行
    btn_beginsave=new QPushButton( "开始保存" );
    btn_stopsave=new QPushButton( "停止保存" );
    btn_stopsave->setStyleSheet("background-color: rgb(255, 0, 0);");
    button_layout1->addWidget( btn_beginsave );
    button_layout1->addWidget( btn_stopsave);
    topic_layout->addLayout( button_layout1);

    QHBoxLayout* button_layout2 = new QHBoxLayout;
    btn_load=new QPushButton( "读取轨迹" );
    btn_clear=new QPushButton( "清除轨迹" );
    button_layout2->addWidget( btn_load );
    button_layout2->addWidget( btn_clear);
    topic_layout->addLayout( button_layout2);

    
    //btn_load=new QPushButton( "读取轨迹" );
    
    // QHBoxLayout* layout = new QHBoxLayout;
    // layout->addLayout( topic_layout );
    // setLayout( layout );

    //QVBoxLayout* topic_layout = new QVBoxLayout;
    setLayout( topic_layout );

    // 创建一个定时器，用来定时发布消息
    QTimer* output_timer = new QTimer( this );

    // 设置信号与槽的连接
    // 输入topic命名，回车后，调用updateTopic()
    // connect( output_topic_editor_, SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));    
    // 输入线速度值，回车后，调用update_Linear_Velocity()    
    // connect( output_topic_editor_1, SIGNAL( editingFinished() ), this, SLOT( update_Linear_Velocity() )); 
    // 输入角速度值，回车后，调用update_Angular_Velocity()
    // connect( output_topic_editor_2, SIGNAL( editingFinished() ), this, SLOT( update_Angular_Velocity() ));

    // 设置定时器的回调函数，按周期调用sendVel()
    // connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendVel() ));

    // 设置定时器的周期，100ms
    // output_timer->start( 100 );

    connect( btn_beginsave, SIGNAL(clicked()), this, SLOT(btn_beginsave_onclick()));
    connect( btn_stopsave, SIGNAL(clicked()), this, SLOT(btn_stopsave_onclick()));
    connect( btn_load, SIGNAL(clicked()), this, SLOT(btn_load_onclick()));
    connect( btn_clear, SIGNAL(clicked()), this, SLOT(btn_clear_onclick()));

    // velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>( "cmd_vel", 1 );
    // path_pub = nh_.advertise<nav_msgs::Path>( "mypath", 1 );
}

void TeleopPanel::btn_beginsave_onclick()
{
    if(saveflag)  return;
    saveflag=true;
    btn_beginsave->setStyleSheet("background-color: rgb(0, 255, 0);");
    btn_stopsave->setStyleSheet("background-color: rgb(123,123, 123);");
    ros::param::set("/gps_pro/saveflag", saveflag);
}

void TeleopPanel::btn_stopsave_onclick()
{
    char dir[200];
    GetPackagePath("gps", dir);
    dir[strlen(dir) - 1] = 0;
    sprintf(dir, "%s/pathdata/", dir);

    if(!saveflag)  return;
    else
    {
        // printf("wsk\n");
        QString fileName = QFileDialog::getSaveFileName(this, tr("open file"), dir, tr("txt(*.txt);;Allfile(*.*)"));
        printf("filename=%s\n", fileName.toStdString().c_str());
        if (fileName != "")   ros::param::set("/gps_pro/savefilename", fileName.toStdString());
    }

    saveflag=false;
    btn_stopsave->setStyleSheet("background-color: rgb(255, 0, 0);");
    btn_beginsave->setStyleSheet("background-color: rgb(123,123, 123);");
    ros::param::set("/gps_pro/saveflag", saveflag);
}



void TeleopPanel::btn_load_onclick()
{
    char dir[200];
    GetPackagePath("gps",dir);
    dir[strlen(dir)-1]=0;
    sprintf(dir,"%s/pathdata/",dir);

    QString fileName = QFileDialog::getOpenFileName(this, tr("open file"), dir, tr("txt(*.txt);;Allfile(*.*)"));
    if(fileName!="")  ros::param::set("/gps_pro/loadfilename", fileName.toStdString());
}

void TeleopPanel::btn_clear_onclick()
{
    ros::param::set("/gps_pro/loadfilename", "clear");
}


} // end namespace rviz_teleop_commander

// 声明此类是一个rviz的插件
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_teleop_commander::TeleopPanel,rviz::Panel )
// END_TUTORIAL
