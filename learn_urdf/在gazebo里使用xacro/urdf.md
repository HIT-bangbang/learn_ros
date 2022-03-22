# xacro/urdf+gazebo

## gazebo原生格式是sdf文件，可以将urdf转换成gazebo

## 注意， 当 URDF 需要与 Gazebo 集成时，和 Rviz 有明显区别:

### 1.必须使用 collision 标签
因为既然是仿真环境，那么必然涉及到碰撞检测，collision 提供碰撞检测的依据。
如果机器人link是标准的几何体形状，和link的 visual 属性设置一致即可。


### 2.必须使用 inertial 标签
此标签标注了当前机器人某个刚体部分的惯性矩阵，用于一些力学相关的仿真计算。
惯性矩阵的设置需要结合link的质量与外形参数动态生成，标准的球体、圆柱与立方体的惯性矩阵公式见mbot_head_gazebo.xacro(已经封装为 xacro 实现):

这样一来，visual  collision  inertial三个标签就都要使用了

### 3.颜色设置
也需要重新使用 gazebo 标签标注，因为之前的颜色设置为了方便调试包含透明度，仿真环境下没有此选项。
在rviz中的颜色设置为：
```xml
        <material name="orange">
        <color rgba="${255/255} ${108/255} ${10/255} 1.0"/>
    </material>
```
在gazebo中，需要在link标签外使用如下设置：
```xml
    <gazebo reference="link1">
        <material>Gazebo/Orange</material>
    </gazebo>
```
可以同时保留上面两个颜色设置，不会报错，并且满足同时在gazebo和rviz中显示的需要。见代码  mycar.urdf

PS：material 标签中，设置的值区分大小写，颜色可以设置为 Red Blue Green Black .....

### 4.如果需要把机器人固定到某个位置（平面上），加入下面的语句：
```xml
    <!-- Used for fixing robot to Gazebo 'base_link' -->
    <link name="world"/>
    
    <joint name="fixed" type="fixed">
        <parent link="world"/>
        <child link="link1"/>
    </joint>
```
感觉一般是用在机械臂上面

## 示例：

见gazebo/mbot_gazebo.xacro