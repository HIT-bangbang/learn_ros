# xacro文件

## 1.使用xacro文件

### 方式1:先将 xacro 文件转换出 urdf 文件，然后集成

先将 xacro 文件解析成 urdf 文件:

    rosrun xacro xacro xxx.xacro > xxx.urdf

然后再按照之前的集成方式直接整合 launch 文件

### 方式2：在 launch 文件中直接加载 xacro(建议使用)
    <param name="robot_description" command="$(find xacro)/xacro $(find urdf_helloworld)/xacro/base.xacro" />

加载robot_description时使用command属性，属性值就是调用 xacro 功能包的 xacro 程序直接解析 xacro 文件。

## 语法详解
### 1.property(属性)

属性定义：
    <xacro:property name="xxxx" value="yyyy" />

属性调用：

    ${name}

算数运算：

    ${数学表达式}

例子：
在urdf中的建立一个base如下：
```xml
    <link name="base_link">
        <visual>
        <geometry>
            <cylinder length="0.6" radius="0.2"/>
        </geometry>
        <material name="blue"/>
        </visual>
        <collision>
        <geometry>
            <cylinder length="0.6" radius="0.2"/>
        </geometry>
        </collision>
    </link>
```
在xacro文件中可以这样写：
```xml
    <xacro:property name="width" value="0.2" />
    <xacro:property name="bodylen" value="0.6" />
    <link name="base_link">
        <visual>
            <geometry>
                <cylinder radius="${width}" length="${bodylen}"/>
            </geometry>
            <material name="blue"/>
        </visual>
        <collision>
            <geometry>
                <cylinder radius="${width}" length="${bodylen}"/>
            </geometry>
        </collision>
    </link>
```
The value of the contents of the \${} construct are then used to replace the \${}. This means you can combine it with other text in the attribute. 

\${}里的内容将会直接被替换（类似于宏）所以可以使用这一特性和别的文本结合。例如：
```xml
    <xacro:property name=”robotname” value=”marvin” />
    <link name=”${robotname}s_leg” />

    上面的内容等价于：
 
    <link name=”marvins_leg” />
```

### 2.数学运算
可以使用四个基本运算（+、-、*、/）、一元减号和括号在\${}构造中构建任意复杂的表达式。

You can build up arbitrarily complex expressions in the \${} construct using the four basic operations (+,-,*,/), the unary minus, and parenthesis. Examples: 

```xml
    <cylinder radius="${wheeldiam/2}" length="0.1"/>
     <origin xyz="${reflect*(width+.02)} 0 0.25" />

    All of the math is done using floats, hence 
    
    <link name="${5/6}"/>
    
    evaluates to 
    
     <link name="0.833333333333"/>
```
### 3.Macros(宏)
一个简单的例子：
```xml
    <xacro:macro name="default_origin">
        <origin xyz="0 0 0" rpy="0 0 0"/>
    </xacro:macro>
    <xacro:default_origin />
```
(This is useless, since if the origin is not specified, it has the same value as this.) This code will generate the following.:
```xml
     <origin rpy="0 0 0" xyz="0 0 0"/>
```
注意：
1、 从技术上讲，该名称不是必需的元素，但您需要指定它才能使用它。 
2、<xacro:$NAME/>的每个实例都被xacro:macro标记的内容替换。
3、请注意，尽管它并不完全相同（rpy和xyz这两个属性的顺序发生了切换），但生成的XML是等效的。
4、 如果找不到具有指定名称的xacro，它将不会展开，也不会生成错误。不会报错！！！！

参数化的宏：

```xml
    <xacro:macro name="default_inertial" params="mass">
        <inertial>
                <mass value="${mass}" />
                <inertia ixx="1.0" ixy="0.0" ixz="0.0"
                    iyy="1.0" iyz="0.0"
                    izz="1.0" />。
        </inertial>
    </xacro:macro>
```

调用时通过指定params的值来参数化宏：

```xml
    <xacro:default_inertial mass="10"/>
```


#### 也可以将整个块作为参数  ??????????

You can also use entire blocks as parameters too.

```xml
    <xacro:macro name="blue_shape" params="name *shape">
        <link name="${name}">
            <visual>
                <geometry>
                    <xacro:insert_block name="shape" />
                </geometry>
                <material name="blue"/>
            </visual>
            <collision>
                <geometry>
                    <xacro:insert_block name="shape" />
                </geometry>
            </collision>
        </link>
    </xacro:macro>

    <xacro:blue_shape name="base_link">
        <cylinder radius=".42" length=".01" />
    </xacro:blue_shape>
```

To specify a block parameter, include an asterisk before its parameter name.
A block can be inserted using the insert_block command
Insert the block as many times as you wish. 

## 实操
在小车底盘的基础上增加雷达和摄像头
代码见mbot_sensor.xacro和mbot_sensor_xacro.launch












