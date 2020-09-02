# 单线激光雷达SLAM建图与路径规划

使用的硬件：autolabor pro1小车、小觅双目相机(S1030标准版本)、IntelNUC迷你主机、显示器、2D激光雷达 Delta-1A  

使用的软件：ubuntu 16.04 LTS、ROS-kinetic、小觅驱动、autolabor pro1小车驱动、Delta-1A驱动、VINS-Fusion算法、ROS-navigation导航包、gmapping建图算法、cartographer_ros建图算法  

## tf介绍
tf变换：map(地图坐标系)-->odom(里程计坐标系)——>base_link(小车基座坐标系)——>传感器坐标系(lidar、camera)    
       ---- map-->odom 由建图算法提供  
       ---- odom --> base_link 由小车驱动提供  
       ---- base_link --> 传感器坐标系 根据传感器在小车上安装的位置确定，使用static_transform_publisher静态坐标转换来设置   

所需传感器数据类型: sensor_msgs/LaserScan  使用Delta-1A驱动发布的雷达扫描话题，话题名：/scan    
所需里程计信息数据类型：nav_msgs/Odometry   使用autolabor pro1发布的里程计，话题名：/wheel_odom    


## 建图
采用gmapping与 cartographer_ros建图算法分别进行建图  

### gmapping建图：(雷达+小车)
打开终端，执行如下指令:  
$ cd ~/catkin_ws_lidar_slam/  
$ source devel/setup.bash  
$ roslaunch autolabor_box_launch create_map_gmapping.launch   ####打开小车urdf模型、驱动、键盘控制节点、打开雷达驱动、打开gmapping节点、打开rviz可视化节点  
####此时使用键盘的上下左右键控制小车移动，开始建图  
####建图完成后，开启新的终端执行如下指令，保存地图：  
$ rosrun map_server map_saver -f map_name    ####保存地图  map_name为保存地图的名称，运行结果，会生成2个文件，后缀名分别为 .pgm  .yaml  


### cartographer_ros建图:(雷达+小车)
##打开终端，执行如下指令：  
$ cd ~/catkin_ws_lidar_slam/  
$ source devel/setup.bash  
$ roslaunch autolabor_box_launch create_map_cartographer.launch  
####此时使用键盘的上下左右键控制小车移动，开始建图  
####建图完成后，开启新的终端执行如下指令，保存地图：  
$ rosrun map_server map_saver -f map_name   


### cartographer_ros建图:(雷达+小车+IMU)
$ cd ~/catkin_ws_lidar_slam/  
$ roslaunch autolabor_box_launch create_map_cartographer_imu.launch  
####此时使用键盘的上下左右键控制小车移动，开始建图  
####建图完成后，开启新的终端执行如下指令，保存地图：  
$ rosrun map_server map_saver -f map_name   


## 定位与路径规划 
分别使用gmapping建图算法与cartographer_ros建图算法建立的地图进行定位与路径规划  

### gmapping算法建立的地图进行定位与路径规划
打开终端，执行如下指令：  
$ cd ~/catkin_ws_lidar_slam/  
$ source devel/setup.bash  
$ roslaunch autolabor_box_launch gmap_navigation.launch     ####打开小车urdf模型、驱动、键盘控制节点、打开雷达驱动、打开map_server节点、打开move_base节点、打开acml节点、打开rviz可视化节点  

使用Rviz工具栏中的2D Nav Goal按钮选择Goal，进行路径规划导航。   

### 使用cartographer_ros算法建立的地图进行定位与路径规划
##打开终端，执行如下指令：  
$ cd ~/catkin_ws_lidar_slam/  
$ source devel/setup.bash  
$ roslaunch autolabor_box_launch carto_navigation.launch  

使用Rviz工具栏中的2D Nav Goal按钮选择Goal，进行路径规划导航。  


------------注意事项---------------  
开是路径规划之前，先用键盘控制小车在原地旋转几圈，加快粒子群收敛，便于小车找准在地图中的位置以及方位。  
-----------注意事项---------------  

