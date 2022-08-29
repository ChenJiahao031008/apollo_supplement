# Apollo纵向控制模块梳理

## 1 纵向控制器参数表

```
lon_controller_conf {
  ts: 0.01			# 控制模块的运行周期
  brake_minimum_action: 0.0  # 百分数, 后续通过取max(brake_minimum_action, vehicle_param_.brake_deadzone()) 确定刹车动作的下边界 
  throttle_minimum_action: 0.0 # 百分数, 后续通过取max(vehicle_param_.throttle_deadzone, throttle_minimum_action)确定油门动作的下边界
  speed_controller_input_limit: 0.8		# 速度控制器的最大输入(m/s)
  station_error_limit: 2.0			# 位置控制器的最大输入(m), 纵向位置误差的最大值
  preview_window: 20.0		# preview_time(预览时间)=preview_window*采样时间ts
  standstill_acceleration: -0.3	# 在倒挡时确定当前汽车的减加速度, max(acceleration_cmd,-lon_controller_conf.standstill_acceleration())
  enable_reverse_leadlag_compensation: false	# 是否允许使用超前/滞后补偿控制器
  station_pid_conf {	# 位置PID控制器参数配置
    integrator_enable: false	# 是否允许打开积分控制器，这里是false,　因此默认位置控制器是P控制器
    integrator_saturation_level: 0.3
    kp: 0.2
    ki: 0.0
    kd: 0.0
  }
  low_speed_pid_conf {	# 低速PID控制器配置
    integrator_enable: true  
    integrator_saturation_level: 0.3  # 积分饱和上限
    kp: 2.0
    ki: 0.3
    kd: 0.0
  }
  high_speed_pid_conf {	# 高速PID控制器配置
    integrator_enable: true
    integrator_saturation_level: 0.3
    kp: 1.0
    ki: 0.3
    kd: 0.0
  }
  switch_speed: 3.0		# 控制算法根据当前的行驶速度来决定选择低速或是高速PID
  switch_speed_window: 1.0	# 没有用到
  reverse_station_pid_conf {	# 倒车档的位置PID配置
    integrator_enable: true
    integrator_saturation_level: 0.5
    kp: 0.4
    ki: 0.1
    kd: 0.0
  }
  reverse_speed_pid_conf {		# 倒车档的速度PID配置
    integrator_enable: true
    integrator_saturation_level: 0.5	# 内部饱和状态界限(上下界)
    kp: 0.8
    ki: 0.3
    kd: 0.0
  }
  reverse_station_leadlag_conf {	# 倒车档的位置超前/滞后补偿器配置
    innerstate_saturation_level: 1000	
    alpha: 1.0	# 滞后系数
    beta: 1.0 # 超前系数
    tau: 0.0 # 时间系数
  }
  reverse_speed_leadlag_conf {		# 倒车档的速度超前/滞后补偿器配置
    innerstate_saturation_level: 1000
    alpha: 1.0
    beta: 1.0
    tau: 0.0
  }
  pitch_angle_filter_conf {		# 数字滤波器的配置
    cutoff_freq: 5	# 截止频率
  }
  calibration_table {	#　油门刹车标定表的配置
    calibration {
      speed: 0.0	# 速度
      acceleration: -1.43 # 加速度
      command: -35.0  # 控制指令
    }
  	...
 }
```

## 2 PID控制器

### 2.1 PID主程序流程

 1. 判断步长是否小于零，若小于零则警告，并返回上一输出`previous_output_`；

 2. 判断是否是`first_hit_`，若是则将`first_hist_`置为false，否则计算
    $$
    diff=(error−error_{previous})/dt
    $$

	3. 判断积分器使能`integrator_enabled_`，假则将`integral_`置零，真则计算`integral_`，在积分前应用 Ki 以避免在稳态时改变 Ki 时出现阶跃；
    $$
    integral+=error∗dt∗ki
    $$

	4. 积分幅值判断：高于积分饱和上限，则等于积分饱和上限，并将积分饱和状态置1；低于积分饱和下限，则等于积分饱和下限，并将积分饱和状态置-1；其余情况积分饱和状态置0；

	5. 保留这一时刻误差，`previous_error_ = error`；输出u幅值判断：高于输出饱和上限，则等于输出饱和上限，并将输出饱和状态置1；低于输出饱和下限，则等于输出饱和下限，并将输出饱和状态置-1；其余情况输出饱和状态置0；

	6. 计算输出值，如下

$$
output=error∗kp​+integral​+diff∗kd
$$

7. 保留这一时刻输出，`previous_output_ = output`



### 2.2 PID标定

​	基于高速控制器->低速控制器->站控制器的顺序

#### 高/低速控制器的调谐

高速控制器代码主要用于跟踪高于某一速度值的期望速度。例如：

```
high_speed_pid_conf {
  integrator_enable: true
  integrator_saturation_level: 0.3
  kp: 1.0
  ki: 0.3
  kd: 0.0
}
```

1. 首先将`kp`, `ki`, 和 `kd` 的值设为0.
2. 然后开始增加`kp`的值，以减小阶跃响应对速度变化的上升时间。
3. 最后，增加`ki`以降低速度控制器稳态误差。

一旦获得较高速度的相对准确的速度跟踪性能，就可以开始从起点开始调整低速PID控制器以获得一个舒适的加速率。

```
low_speed_pid_conf {
  integrator_enable: true
  integrator_saturation_level: 0.3
  kp: 0.5
  ki: 0.3
  kd: 0.0
}
```

*注意:*  当设备切换到 *Drive*时，Apollo 通常将速度设置为滑行速度。

#### 位置PID控制器调谐

Apollo 使用车辆的位置PID控制器来跟踪车辆轨迹基准与车辆位置之间的站误差。  一个位置PID控制器调谐示例如下所示

```
low_speed_pid_conf {
  integrator_enable: true
  integrator_saturation_level: 0.3
  kp: 0.5
  ki: 0.3
  kd: 0.0
}
```

调参过程同上述速度PID控制器的调参过程一致



## 3 油门刹车标定表

在Apollo系统中，控制模块会请求加速度量值。通过车辆标定表，控制模块便能找到准确产生所需加速度量值对应的油门、刹车踏板开合度控制命令，之后下发给车辆底盘。车辆标定表提供一个描述车辆速度、油门／刹车踏板开合度、加速度量之间关系的映射表。油门刹车标定过程便是生成车辆标定表的过程。



## 4 lead/lag补偿器

### leg/leg补偿器及其参数

$$
H(s)=β​\frac{τs+1}{ταs+1}​
$$

$$
其中， \alpha为滞后系数； \beta为超前系数； \tau为给定的时间系数
$$

采用双线性变换，T为采样周期
$$
s=\frac{2}{T}​\frac{z−1}{z+1}​
$$
带入s进入传递函数，可得
$$
H(z)=β \frac{2τ(z−1)+T(z+1)}{2τα(z−1)+T(z+1)}​
$$

$$
H(z)=\frac{(Tβ−2βτ)+(2βτ+Tβ)z}{(T−2ατ)+(2ατ+T)z}​
$$

$$
H(z)=\frac{kn_0​+kn_1​∗z}{kd_0​+kd_1​∗z}​
$$

对应程序中的代码，如下
$$
a1=ατ, a0=1.00， b1 = βτ, b0 = β;
$$

$$
kn1​=2βτ+Tβ, k n 0 = T β − 2 β τ, kd_1 = 2 \alpha \tau + T， k d 0 = T − 2 α τ
$$

```
void LeadlagController::TransformC2d(const double dt) {
  if (dt <= 0.0) {
    AWARN << "dt <= 0, continuous-discrete transformation failed, dt: " << dt;
    transfromc2d_enabled_ = false;
  } else {
    // 公式？
    double a1 = alpha_ * tau_;
    double a0 = 1.00;
    double b1 = beta_ * tau_;
    double b0 = beta_;
    Ts_ = dt;
    // 带入默认参数数值后，dt，dt，dt，dt
    kn1_ = 2 * b1 + Ts_ * b0;
    kn0_ = Ts_ * b0 - 2 * b1;
    kd1_ = 2 * a1 + Ts_ * a0;
    kd0_ = Ts_ * a0 - 2 * a1;
    if (kd1_ <= 0.0) {
      AWARN << "kd1 <= 0, continuous-discrete transformation failed, kd1: "
            << kd1_;
      transfromc2d_enabled_ = false;
    } else {
      transfromc2d_enabled_ = true;
    }
  }
}
```

### lead/leg补偿器原理

$lead/lag$主要是超前或滞后(不能同时)，本质就是讨论零点 $\frac{1}{\tau}$和极点 $\frac{1} {α τ}$ 的前后位置，如下：

- 若零点$$\frac{1}{\tau}$$极点$$\frac{1}{\alpha \tau}$$，即 $α < 1$，此时为超前校正，作用是提高响应速度，避免引入高频震荡；


- 若零点 $\frac{1}{\tau}$ <极点 $\frac{1}{\alpha \tau}$，即 $\alpha >1$ ，此时为滞后校正，作用是提高稳态精度，减少稳态误差，但暂态响应将变慢

关于超前/滞后补偿器的原理分析推荐:

滞后补偿器：https://www.bilibili.com/video/BV1W7411n7G8?spm_id_from=333.337.search-card.all.click

超前补偿器:   https://www.bilibili.com/video/BV1JJ411i7ph?spm_id_from=333.999.0.0

对于增益系数 $ \beta$，本质上就是讨论$lead/lag$模块串入系统后对系统开环放大系数的影响，有如下情况：

- 若$0<\beta<1$，会使得系统的开环放大系数下降，即幅值衰减；
- 若$\beta>1$，会使得系统的开环放大系数上升，即幅值增加

增益系数 $β$ 的分析: https://www.bilibili.com/video/BV14J411A7M2?spm_id_from=333.999.0.0

### lead/leg补偿器程序流程

1. 判断连续转离散(`transfromc2d_enabled_`)是否成功，失败则重新进行`TransformC2d`，如果再次失败，则发出警告`C2d transform failed; will work as a unity compensator`，并返回错误；

2. 检查步长`dt`是否小于等于零。如果小于等于零，则发出警告`dt <= 0, will use the last output`，返回上一时刻输出`previous_output_`；

3. 计算内部状态`innerstate_`，计算公式如下
   $$
   s_{inner}(k)=\frac {e(k)−s_{inner}(k−1)∗kd_0}{kd_1}
   $$
   移位可得公式
   $$
   s_{inner}​(k−1)∗kd_0+s_{inner}​(k)∗kd_1=e(k)
   $$
   上述公式不能直观理解，需要进行转换

   观察离散传递函数，如下
   $$
   \frac {u(z)}{y(z)}​=\frac {kn_0​+kn_1​z​}{kd_0​+kd_1​z}
   $$

   $$
   y(z)=\frac {kn_0​+kn_1​z}{kd_0​+kd_1​z}​u(z)
   $$

   令
   $$
   x(z)=\frac {1}{kd_0​+kd_1​z}​u(z)
   $$
   即

$$
kd_0​x(k)+kd_1​x(k+1)=u(k)
$$

$$
x(k+1)=\frac {u(k)−kd_0​x(k)}{kd_1}
$$

​		传递函数变为
$$
y(z)=(kn_0​+kn_1​z)x(z)
$$
​		从而得到
$$
y(k)=kn_0​x(k)+kn_1​x(k+1)
$$

$$
其中，y(k)为output, x(k)为innerstate，u(k)为error(即系统输出与参考值的偏差)，并按照x(k+1) = \frac{ u(k) - kd_0x(k) }{kd_1}计算innerstate的值
$$

4. 进行`innerstate`幅值判断：高于状态饱和上限，则等于状态饱和上限，并将状态饱和状态置1；低于状态饱和下限，则等于状态饱和下限，并将状态饱和状态置-1；其余情况状态饱和状态置0；
5. 计算`output`
6. 保存`innerstate`和`output`变量，即为`previous_innerstate`和`previous_output`
