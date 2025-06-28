  <!-- 配置过程 -->
 需要将 models 文件拷贝到  主目录隐藏文件.gazebo文件夹下面
 
  <!-- Gazebo 报错不影响正常使用 -->
 [ERROR] [1747124894.345114825]: No p gain specified for pid.  Namespace: /mobile_base/gazebo_ros_control/pid_gains/rear_right_wheel_joint

 <!-- 总运行文件 -->
 source devel/setup.bash
 roslaunch pp_controller run_sim_all.launch




