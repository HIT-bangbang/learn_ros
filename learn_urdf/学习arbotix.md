# arbotix
## 简介
arbotix是一款控制电机、舵机的控制板，并提供了ROS功能包。他可以驱动真实的arbotix控制板，也可以在rviz中仿真。

简单的说，arbotix节点接受 /cmd_vel topic ,并驱动小车运动。

## 使用
### 安装

    sudo apt-get install ros-<<VersionName()>>-arbotix

### 创建配置文件，后缀为yml

```yaml
        # 该文件是控制器配置,一个机器人模型可能有多个控制器，比如: 底盘、机械臂、夹持器(机械手)....
        # 因此，根 name 是 controller
        controllers: {
        # 单控制器设置
        base_controller: {
                #类型: 差速控制器
            type: diff_controller,
            #参考坐标
            base_frame_id: base_footprint, 
            #两个轮子之间的间距
            base_width: 0.2,
            #控制频率
            ticks_meter: 2000, 
            #PID控制参数，使机器人车轮快速达到预期速度
            Kp: 12, 
            Kd: 12, 
            Ki: 0, 
            Ko: 50, 
            #加速限制
            accel_limit: 1.0 
            }
        }
```

### 在launch 文件中添加arbotix节点的支持

```xml
    <node name="arbotix" pkg="arbotix_python" type="arbotix_driver" output="screen">
        <rosparam file="$(find my_urdf05_rviz)/config/hello.yaml" command="load" />
        <param name="sim" value="true" />
    </node>
```
代码解释:

<node> 调用了 arbotix_python 功能包下的 arbotix_driver 节点

<rosparam> arbotix 驱动机器人运行时，需要获取机器人信息，可以通过 file 加载配置文件

<param> 在仿真环境下，需要配置 sim 为 true

### 启动仿真，通过rostopic，/cmd_vel发送消息

    rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.2, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.5}}'


## 要注意的
### 1.arbotix需要base_footprint到base_link的TF变换（可以在launch文件中发布静态的，也可以在urdf文件里面指定）

https://blog.csdn.net/qq_43481884/article/details/105433575

### 2.警告：Joint state with name: “base_l_wheel_joint“ was received but not found in URDF
这个问题可能是arbotix对于joint的命名有自己的规定，可以修改urdf/xacro文件里面的joint名称
name=“left”改成name="base_l_wheel_joint"，name="${name_wheel2base}"中的_wheel2base删掉之后，再次运行还会抛出Joint state with name: "base_r_wheel_joint" was received but not found in URDF，这样的异常和上述一样修改，把左改成右。
本质就是把joint的命名改成警告给的提示才行。

https://blog.csdn.net/qq_41949101/article/details/116665707

## 示例 
见mbot_xacro.launch