<!--
 * @Author: likecanyon 1174578375@qq.com
 * @Date: 2022-05-21 20:33:25
 * @LastEditors: likecanyon 1174578375@qq.com
 * @LastEditTime: 2022-11-08 20:07:06
 * @FilePath: /my_iiwa_stack/readme.md
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
-->
# 注意！
  iiwa_stack 默认接受来自 192.170.10.100主机的控制，所以启动ros master节点之前，在`~/.bashrc`导入这两句话
  并设置网卡对应的ip
  ```bash
   export ROS_MASTER_URI=http://192.170.10.100:11311
   export ROS_IP=192.170.10.100 
  ```
  将与iiwa工控箱连接的网线插入到KONI接口


## 直接使用：(每一步都是独立一个例程)
  1. rviz同步显示，不做任何操作
  `roslaunch my_iiwa_examples  rviz_plot.launch  `
  2. 笛卡尔控制(或关节空间控制，需要roslaunch 文件里use_cartesian_command 参数写为 false)
  `roslaunch my_iiwa_examples  iiwa_command.launch`
  3. 笛卡尔空间阻抗 和关节空间阻抗（运行过程中，终端输入文字，回车切换模式）
  `roslaunch my_iiwa_examples  iiwa_control_mode.launch`

## 其他注意事项：
 
  1. 如果需要更改工具，理论上需要更改sunrise客户端的工具偏移(用于选择运动工具系)  和 URDF文件里工具偏移(用于rviz的显示和iiwa-java的tf查询)
  2. 如果需要实现笛卡尔空间的直线运动或之类，话题的'header.frame_id'字段需要填入数据参考的坐标系，如iiwa_link_0

## 实验效果
[使用Omega遥控iiwa](https://www.bilibili.com/video/BV1KG411w7oW/?pop_share=1&vd_source=53f8b5329a2c2fa2fdc10cdbba494816)