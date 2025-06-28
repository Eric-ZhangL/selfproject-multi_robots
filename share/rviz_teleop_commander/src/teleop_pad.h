#ifndef TELEOP_PAD_H
#define TELEOP_PAD_H

//所需要包含的头文件
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h>   //plugin基类的头文件
#include <QPushButton>
#endif

//class QLineEdit;
// class QPushButton;


namespace rviz_teleop_commander
{
// 所有的plugin都必须是rviz::Panel的子类
class TeleopPanel: public rviz::Panel
{
// 后边需要用到Qt的信号和槽，都是QObject的子类，所以需要声明Q_OBJECT宏
Q_OBJECT
public:
    // 构造函数，在类中会用到QWidget的实例来实现GUI界面，这里先初始化为0即可
    TeleopPanel( QWidget* parent = 0 );

// 公共槽.
public Q_SLOTS:

// 内部槽.
protected Q_SLOTS:
    void btn_beginsave_onclick();
    void btn_stopsave_onclick();
    void btn_load_onclick();
    void btn_clear_onclick();

// 内部变量.
protected:
    QPushButton *btn_beginsave, *btn_stopsave;
    QPushButton *btn_load, *btn_clear;

    bool saveflag;

    // ROS的publisher，用来发布速度topic
    ros::Publisher path_pub;

    // ROS节点句柄
    ros::NodeHandle nh_;
};

} // end namespace rviz_teleop_commander

#endif // TELEOP_PANEL_H
