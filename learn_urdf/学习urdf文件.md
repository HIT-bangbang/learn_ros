# urdf文件语法
## 1.robot标签
urdf 中为了保证 xml 语法的完整性，使用了robot标签作为根标签，所有的 link 和 joint 以及其他标签都必须包含在 robot 标签内,在该标签内可以通过 name 属性设置机器人模型的名称
### (1).属性

    name: 指定机器人模型的名称
### (2).子标签

    其他标签都是子级标签

## 2.link
urdf 中的 link 标签用于描述机器人某个部件(也即刚体部分)的外观和物理属性，比如: 机器人底座、轮子、激光雷达、摄像头...每一个部件都对应一个 link, 在 link 标签内，可以设计该部件的形状、尺寸、颜色、惯性矩阵、碰撞参数等一系列属性
### (1).属性

    name ---> 为连杆命名

### (2).子标签

    visual ---> 描述外观(对应的数据是可视的)

        geometry 设置连杆的形状

            标签1: box(盒状)
                属性:size=长(x) 宽(y) 高(z)

            标签2: cylinder(圆柱)
                属性:radius=半径 length=高度

            标签3: sphere(球体)
                属性:radius=半径

            标签4: mesh(为连杆添加皮肤)
                属性: filename=资源路径(格式:package://<packagename>/<path>/文件)

        origin 设置偏移量与倾斜弧度

            属性1: xyz=x偏移 偏移 z偏移

            属性2: rpy=x翻滚 y俯仰 z偏航 (单位是弧度)

        metrial 设置材料属性(颜色)

            属性: name

            标签: color
                属性: rgba=红绿蓝权重值与透明度 (每个权重值以及透明度取值[0,1])

    collision ---> 连杆的碰撞属性（在gazebo中使用是必需的）

    Inertial ---> 连杆的惯性矩阵（在gazebo中使用是必需的）

### (3)案例：

```   xml
    <link name="base_link">
        <visual>
            <!-- 形状 -->
            <geometry>
                <!-- 长方体的长宽高 -->
                <!-- <box size="0.5 0.3 0.1" /> -->
                <!-- 圆柱，半径和长度 -->
                <!-- <cylinder radius="0.5" length="0.1" /> -->
                <!-- 球体，半径-->
                <!-- <sphere radius="0.3" /> -->
            </geometry>
            <!-- xyz坐标 rpy翻滚俯仰与偏航角度(3.14=180度 1.57=90度) -->
            <origin xyz="0 0 0" rpy="0 0 0" />
            <!-- 颜色: r=red g=green b=blue a=alpha -->
            <material name="black">
                <color rgba="0.7 0.5 0 0.5" />
            </material>
        </visual>
    </link>
```

## 3.joint

urdf 中的 joint 标签用于描述机器人关节的运动学和动力学属性，还可以指定关节运动的安全极限，机器人的两个部件(分别称之为 parent link 与 child link)以"关节"的形式相连接，不同的关节有不同的运动形式: 旋转、滑动、固定、旋转速度、旋转角度限制....,比如:安装在底座上的轮子可以360度旋转，而摄像头则可能是完全固定在底座上。

joint标签对应的数据在模型中是不可见的

### (1).属性

    name ---> 为关节命名

    type ---> 关节运动形式

        continuous: 旋转关节，可以绕单轴无限旋转

        revolute: 旋转关节，类似于 continues,但是有旋转角度限制

        prismatic: 滑动关节，沿某一轴线移动的关节，有位置极限

        planer: 平面关节，允许在平面正交方向上平移或旋转

        floating: 浮动关节，允许进行平移、旋转运动

        fixed: 固定关节，不允许运动的特殊关节

### (2).子标签

    parent(必需的)

    parent link的名字是一个强制的属性：
        link:父级连杆的名字，是这个link在机器人结构树中的名字。

    child(必需的)

    child link的名字是一个强制的属性：
        link:子级连杆的名字，是这个link在机器人结构树中的名字。

    origin
        属性: xyz=各轴线上的偏移量 rpy=各轴线上的偏移弧度。

    axis
        属性: xyz用于设置围绕哪个关节轴运动。

### (3).案例

需求:创建机器人模型，底盘为长方体，在长方体的前面添加一摄像头，摄像头可以沿着 Z 轴 360 度旋转。

urdf文件如下：

```xml
<!-- 
    需求: 创建机器人模型，底盘为长方体，
         在长方体的前面添加一摄像头，
         摄像头可以沿着 Z 轴 360 度旋转
 -->
<robot name="mycar">
    <!-- 底盘 -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.5 0.2 0.1" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <material name="blue">
                <color rgba="0 0 1.0 0.5" />
            </material>
        </visual>
    </link>

    <!-- 摄像头 -->
    <link name="camera">
        <visual>
            <geometry>
                <box size="0.02 0.05 0.05" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <material name="red">
                <color rgba="1 0 0 0.5" />
            </material>
        </visual>
    </link>

    <!-- 关节 -->
    <joint name="camera2baselink" type="continuous">
        <parent link="base_link"/>
        <child link="camera" />
        <!-- 需要计算两个 link 的物理中心之间的偏移量 -->
        <origin xyz="0.2 0 0.075" rpy="0 0 0" />
        <axis xyz="0 0 1" />
    </joint>
</robot>
```
launch文件如下：
```xml
<launch>
    <!--robot_description参数不可以缺少，此处参数的值需要修改-->
    <param name="robot_description" textfile="$(find urdf_rviz_demo)/urdf/urdf/urdf03_joint.urdf" />
        <!--打开rviz并使用本地配置文件，此处参数的值需要修改-->
    <node pkg="rviz" type="rviz" name="rviz" args="-d $(find urdf_rviz_demo)/config/helloworld.rviz" /> 
    <!--如果只是要打开rviz，这样写：
    <node pkg="rviz" type="rviz" name="rviz" />        
    -->
    <!-- 添加关节状态发布节点 -->
    <node pkg="joint_state_publisher" type="joint_state_publisher" name="joint_state_publisher" />
    <!-- 添加机器人状态发布节点 -->
    <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" />
    <!-- 可选:用于控制关节运动的节点 -->
    <node pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" name="joint_state_publisher_gui" />

</launch>
```
# 练习

需求描述:
创建一个四轮圆柱状机器人模型，机器人参数如下,底盘为圆柱状，半径 10cm，高 8cm，四轮由两个驱动轮和两个万向支撑轮组成，两个驱动轮半径为 3.25cm,轮胎宽度1.5cm，两个万向轮为球状，半径 0.75cm，底盘离地间距为 1.5cm(与万向轮直径一致)

实现流程:

    创建机器人模型可以分步骤实现

    1.新建 urdf 文件，并与 launch 文件集成

    2.搭建底盘

    3.在底盘上添加两个驱动轮

    4.在底盘上添加两个万向轮

### 代码见本文件夹内的mbot.urdf以及mbot_urdf.launch

# URDF工具

## (1)check_urdf 语法检查
check_urdf  urdf_filename

## (2)urdf_to_graphiz 结构查看

进入urdf文件所属目录，调用:urdf_to_graphiz urdf文件，当前目录下会生成 pdf 文件