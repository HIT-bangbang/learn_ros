# odom，map，baselink之间的关系
最常用的就是map，odom，base_link，base_laser，base_footprint

map:地图坐标系，顾名思义，一般设该坐标系为固定坐标系（fixed frame），一般与机器人所在的世界坐标系一致。

base_link:机器人本体坐标系，与机器人中心重合，当然有些机器人(PR 2)是base_footprint,其实是一个意思。

odom：里程计坐标系，这里要区分开odom topic，这是两个概念，一个是坐标系，一个是根据编码器（或者视觉等）计算的里程计。但是两者也有关系，odom topic 转化得位姿矩阵是odom-->base_link的tf关系。
odom和map坐标系在机器人运动开始是重合的。但是，随着时间的推移是不重合的，而出现的偏差就是里程计的累积误差。

那map-->odom的tf怎么得到?就是在一些校正传感器合作校正的package比如amcl会给出一个位置估计（localization），这可以得到map-->base_link的tf，所以估计位置和里程计位置的偏差也就是odom与map的坐标系偏差。

所以，如果odom计算没有错误，那么map-->odom的tf就是0.

base_laser:激光雷达的坐标系，与激光雷达的安装点有关，其与base_link的tf为固定的。

也就是，通过imu，轮式编码器等计算得到了odom，也就是odom-->base_link的关系
通过amcl等算法，读取激光雷达信息与前期工作中建立的地图对比，得到了map-->base_link，在ros中的表现的acml发布了odom-->map的变换。也就是通过自适应蒙特卡洛得到了里程计的偏差。

如果里程计足够精确（轮式编码器没有偏差，IMU也正确，GPS没有漂移，不打滑等等），同时acml用先验的地图信息和当前激光雷达数据对比算出来的目前机器人位置也足够精确，在这种情况下，odom坐标系和map坐标系应该是重和的，意味两种方式得到的机器人都在同一个位置。但是这是不可能的。里程计会有漂移，acml算法也不一定准确。

slam的目的是最小化（优化）odom和map之间的差距。



通俗解释：

比如说我现在使用轮式编码器算出来了一个一维的坐标X=10（baselink在world坐标系），但是这个位置的真实坐标（acml算出来的）应该是X'=2（baselink'在world坐标系），也就是说我计算的坐标和真实坐标出现了偏差。但是ROS在说的时候换了个说法，不说编码器计算出来的坐标在world中漂了8，而是新创建了两个坐标系odom和map，将baselink和baselink'重合，然后分别在odom和map坐标系里面描述它，这样baselink和baselink'的差距就变成了odom和map之间的差距，即odom坐标系相对于map坐标系漂了8。


如果用IMU作积分的话，通过IMU获得的是odom坐标系下的坐标，初始时odom和map重合，都为0。那么短时间内，由于IMU的漂移很小，所以获得的位移deltaX是准确的，最终的在odom坐标系下的坐标是X=dextaX，在map中的坐标X'也是deltaX。但是时间长了之后，IMU积分开始产生漂移driftX了，最终在map下的坐标X=deltaX+driftX。由于我们定义IMU在短时间内产的的位移deltaX是准确的，所以deltaX是就是IMU在odom坐标系下的真实位移，但是此时odom相对于map则漂移了driftX。

# baselink和basefootprint之间的关系

base_link是与机器人中心重合，而base_footprint是base_link在地面的投影（所以这两者的z坐标才会不一样）。





