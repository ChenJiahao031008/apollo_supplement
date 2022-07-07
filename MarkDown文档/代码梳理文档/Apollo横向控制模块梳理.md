# Apollo横向控制模块梳理

## 1 横向控制器参数表

```
lat_controller_conf {
  ts: 0.01			# 控制器运行周期
  preview_window: 0	# preview_time(预览时间)=preview_window*采样时间ts
  cf: 155494.663	#  前轮侧偏刚度，左右轮之和
  cr: 155494.663	#  后轮侧偏刚度，左右轮之和
  mass_fl: 520		#  左前悬的质量
  mass_fr: 520		#  右前悬的质量
  mass_rl: 520		#  左后悬的质量
  mass_rr: 520		#  右后悬的质量
  eps: 0.01			#  LQR迭代求解精度
  matrix_q: 0.05	#  Q矩阵是LQR中目标函数中各个状态量(X=[e1 e1' e2 e2'])平方和的权重系数
  matrix_q: 0.0
  matrix_q: 1.0
  matrix_q: 0.0
  reverse_matrix_q: 0.05	# 倒车档时,目标函数中各个状态量(X=[e1 e1' e2 e2'])平方和的权重系数
  reverse_matrix_q: 0.0
  reverse_matrix_q: 1.0
  reverse_matrix_q: 0.0
  cutoff_freq: 10		# 截止频率, 用于后续计算得到滤波器的传递函数分子和分母
  mean_filter_window_size: 10	# 均值滤波窗口大小
  max_iteration: 150		# LQR问题求解的最大迭代次数
  max_lateral_acceleration: 5.0		# 最大允许的横向加速度
  enable_reverse_leadlag_compensation: true  # 使能横向控制中的超前滞后控制器, 用于改善闭环反馈系统的响应速度
  enable_steer_mrac_control: false	# 使能mrac模型参考自适应控制
  enable_look_ahead_back_control: true # 使能前进倒车时的预瞄控制
  lookahead_station: 1.4224	# 前进档时汽车控制的预瞄距离
  lookback_station: 2.8448	#　倒车档时汽车控制的预瞄距离
  lookahead_station_high_speed: 1.4224	  #  高速前进预瞄距离，针对非R档
  lookback_station_high_speed: 2.8448	  #  高速前进预瞄距离，针对R档
  lat_err_gain_scheduler {	# 横向误差增益调度表, 在不同速度下, 在不同的速度下为测定的横向误差乘上一个比例系数
    scheduler {
      speed: 4.0	# 速度
      ratio: 1.0	# 比例系数
    }
    scheduler {
      speed: 8.0
      ratio: 0.6
    }
    scheduler {
      speed: 12.0
      ratio: 0.2
    }
    scheduler {
      speed: 20.0
      ratio: 0.1
    }
    scheduler {
      speed: 25.0
      ratio: 0.05
    }
  }
  heading_err_gain_scheduler {	# 朝向误差增益调度表, 在不同速度下, 在不同的速度下为测定的朝向误差乘上一个比例系数
    scheduler {
      speed: 4.0
      ratio: 1.0
    }
    scheduler {
      speed: 8.0
      ratio: 0.6
    }
    scheduler {
      speed: 12.0
      ratio: 0.4
    }
    scheduler {
      speed: 20.0
      ratio: 0.2
    }
    scheduler {
      speed: 25.0
      ratio: 0.1
    }
  }
  reverse_leadlag_conf {	# 倒车档的leg/lag补偿器配置
    innerstate_saturation_level: 3000  # 内部饱和状态界限(上下界)
    alpha: 1.0
    beta: 1.0
    tau: 0.0
  }
  steer_mrac_conf {  		# flase 跳过
    mrac_model_order: 1
    reference_time_constant: 0.09
    reference_natural_frequency: 10
    reference_damping_ratio: 0.9
    adaption_state_gain: 0.0001
    adaption_desired_gain: 0.0001
    adaption_nonlinear_gain: 0.0001
    adaption_matrix_p: 1.0
    mrac_saturation_level: 1.0
    anti_windup_compensation_gain: 0.0001
    clamping_time_constant: 0.08
  }
}
```

## 2 LQR最优控制器

### 2.1 LQR横向控制器原理

$LQR$(linear quadratic regulator)即线性二次型调节器，是常用控制算法的一种。

考虑有如下离散线性系统：
$$
x_{t+1} = Ax_t + Bu_t, x_{init} = x(0)
$$
在此基础上设计一个状态反馈控制器
$$
u = -Kx
$$
为了能使上述控制器达到期望的稳定性性能,将$u = -Kx$带入状态方程组中,有
$$
\dot x = (A - BK)x = A_cx
$$
$LQR$的目标就是找到一组控制量 ![[公式]](https://www.zhihu.com/equation?tex=u_0%2Cu_1%2C...) 使$x_0, x_1, ...$足够小,　即使系统达到稳定状态, $u_0, u_1, ...$足够小, 即花费较小的控制代价.

为了达到上述效果, 定义代价函数:
$$
J = \frac{1}{2}\int_{r=0}^\infty{x^TQx + u^TRu\;dt}
$$
其中，$Q、R$就是需要设计的半正定矩阵和正定矩阵。一般来说，选取$Q$、$R$矩阵的时候，为了方便观察各个系统状态量而选取对角阵，增加$Q$的一个值，意味着这个值作用的系统状态量，将以更快的速度衰减到0，比如， $Q_{11}$选取较大的值，会让 $x_1$很快的衰减到0；另外一方面，加大 $R$的值，会使得对应的控制量减小，控制器执行更少的动作，意味着系统的状态衰减将变慢。

公式推导:

1. 将 $u = − K x$代入代价函数后，有

​	
$$
J = \frac{1}{2}\int_{0}^{\infty} {x^T(Q + K^TRK)}\;dt
$$

	2. 假设纯在一个常量矩阵$P$使得，有

$$
\frac{d(x^TPx)}{dt} = -x^T(Q + K^TRK)x
$$

 3. 将上式带入可知
    $$
    J = -\frac{1}{2}\int_{0}^{\infty}\frac{d(x^TPx)}{dt} = \frac{1}{2}x^T(0)Px(0)
    $$

 4. 

​	即，当$t$趋近于无穷时，系统状态向量$x(t$)趋近于0，这样就直接结算出了积分方程。

 	4. 将左式微分展开


$$
\dot x^TPx + x^TP\dot x + x^TQx +x^TK^TRkx = 0
$$

​		状态变量$x$的微分式带入$\dot x = A_cx$

​	
$$
x^TA_c^TPx + x^TPA_cx + x^TQx + x^tK^tRKx = 0
$$
​		整理方程有

​	
$$
x^T(A_c^TP + PA_c + Q + K^TRK)x = 0
$$
​		如果上述二次型方程要有解，则括号内的部分必须为0

​		即
$$
A_c^TP + PA_c + Q + K^TRK = 0
$$
​		把$A_c = A - BK$代入
$$
A^TP + PA + Q + K^TRK - K^TB^TP - PBK = 0
$$

5. 令$K = R^{-1}B^TP$, 代入
   $$
   A^TP + PA + Q - PBR^{-1}B^TP = 0
   $$
   此时, $A, B, Q, R$都是已知量，可以通过化解得来的方程求解$P$, 此即$Riccati$方程

回顾一下$LQR$算法的思路:

1. 选择参数矩阵$Q,R$
2. 求解$Riccati$方程得到矩阵P
3. 根据P计算 $K = R^{-1}B^{T}P$
4. 计算控制量$u = -Kx$

## 2.2 LQR控制器标定

$LQR$控制器的调谐步骤如下:

1. 首先更新车辆相关的物理参数，如下面的示例所示。然后，按照上面列出的基本$LQR$控制器调整步骤*横向控制器调谐*和定义矩阵$Q$参数。

   ```
   lat_controller_conf {
     cf: 155494.663
     cr: 155494.663
     wheelbase: 2.85
     mass_fl: 520
     mass_fr: 520
     mass_rl: 520
     mass_rr: 520
     eps: 0.01
     steer_transmission_ratio: 16
     steer_single_direction_max_degree: 470
   }
   ```

2. 将`matrix_q` 中所有元素设置为零.

3. 增加`matrix_q`中的第三个元素，它定义了航向误差加权，以最小化航向误差。

4. 增加`matrix_q`的第一个元素，它定义横向误差加权以最小化横向误差。

## 3. 前馈控制

为了更好的消除汽车在运动过程中产生的稳态误差，apollo的控制模块加入了前馈控制来消除这部分的稳态误差，接下来从公式推导的方式说明为了需要加入前馈控制。



### 3.1 状态方程推导

![img](https://pic3.zhimg.com/80/v2-be925eec61e1385d818bd3c417a43aae_720w.jpg)

apollo的控制模块是将汽车模型先简化为自行车模型后再进行分析的.

根据牛顿第二定律结合车辆前后轮受力:
$$
ma_y = F_{yf} + F_{yr}
$$
根据力矩平衡结合车辆前后轮受力和受力点到车辆重心距离，有：
$$
I_z\ddot\psi = l_fF_{yf} - l_rF_{yr}
$$
上述两式中，汽车质量 ![[公式]](https://www.zhihu.com/equation?tex=m) 、转动惯量 ![[公式]](https://www.zhihu.com/equation?tex=I_z) 、前轴到重心距离 ![[公式]](https://www.zhihu.com/equation?tex=l_f) 和后轴到重心距离 ![[公式]](https://www.zhihu.com/equation?tex=l_r) 都是可测量的。为了求解两等式，需要分别求得车辆横向加速度 ![[公式]](https://www.zhihu.com/equation?tex=a_y) 和前轮横向受力 ![[公式]](https://www.zhihu.com/equation?tex=F_%7Byf%7D) 和后轮横向受力 ![[公式]](https://www.zhihu.com/equation?tex=F_%7Byr%7D)

横向加速度可以分解为由横向位移产生的加速度和向心加速度。
$$
a_y = \ddot{y} + V_x\dot\psi
$$
 $y$为横向位移,  $\psi$航向角,  $V_x$为纵向速度。这样横向加速度就分解为位移和航向角的表达式



![img](https://pic2.zhimg.com/80/v2-68be69075365d81eeb355aa8d3c778ad_720w.jpg)



前轮横向受力:
$$
F_{yf} = 2C_{af}(\delta - \theta_{vf})
$$
其中 ![[公式]](https://www.zhihu.com/equation?tex=C_%7Baf%7D) 为前轮侧偏刚度， ![[公式]](https://www.zhihu.com/equation?tex=%5Cdelta) 为前轮转角， ![[公式]](https://www.zhihu.com/equation?tex=%5Ctheta_%7Bvf%7D) 为前轮速度偏角。

后轮横向受力可以近似为:
$$
F_{yr} = 2C_{ar}(-\theta_{vr})
$$


其中 ![[公式]](https://www.zhihu.com/equation?tex=C_%7Bar%7D) 为后轮侧偏刚度， ![[公式]](https://www.zhihu.com/equation?tex=%5Ctheta_%7Bvf%7D) 为后轮速度偏角。

![[公式]](https://www.zhihu.com/equation?tex=tan%28%5Ctheta%29) 等于车辆横向速度比纵向速度，而横向速度由车辆自身横向速度和绕重心转动速度组成。
$$
tan(\theta_{vf}) = \frac{V_y + l_f\dot\psi}{V_x} \\
tan(\theta_{vf}) = \frac{V_y - l_r\dot\psi}{V_x}
$$
在小角度转向假设下，有
$$
\theta_{vf} = \frac{V_y + l_f\dot\psi}{V_x} \\
\theta_{vf} = \frac{V_y - l_r\dot\psi}{V_x}
$$
至此，把变化后的各变量表达式代入$ma_y = F_{yf} + F_{yr}$，有如下推导过程:
$$
ma_y = F_{yf} + F_{yr}  \\
m(\ddot{y} + V_x\dot\psi) = 2C_{af}(\delta - \theta_vf) + 2C_{ar}(-\theta_{vr}) \\
m(\ddot{y} + V_x\dot\psi) = 2C_{af}(\delta - \frac{\dot{y} + l_f\dot\psi}{V_x}) + 2C_{ar}(-\frac{V_y - l_r\dot\psi}{V_x}) \\
\ddot{y} = -\frac{2C_{af} + 2C_{ar}}{mV_x}{\dot{y}} + (-V_x - \frac{2C_{af}l_f - 2C_{ar}l_r}{mV_x})\dot\psi + \frac{2C_{af}}{m}{\delta}
$$
把变化后的各变量表达式代入$I_z\ddot\psi = l_fF_{yf} - l_rF_{yr}$, 有如下推导过程：
$$
I_z\ddot\psi = l_fF_{yf} - l_rF_{yr} \\
I_z\ddot\psi = l_f2C_{af}(\delta - \theta_{vf}) - l_r2C_{ar}(-\theta{vr}) \\
I_z\ddot\psi = l_f2C_{af}(\delta - \frac{V_y + l_f\dot\psi}{V_x}) - l_r2C_{ar}(-\frac{V_y - l_r\dot\psi}{V_x}) \\
\ddot\psi = -\frac{2l_fC_{af} - 2l_rC_{ar}}{I_zV_x}\dot{y} + (-\frac{2l_f^2C_{af} + 2l_r^2C_{ar}}{I_zV_x})\dot\psi + \frac{2l_fC_{af}}{I_z}\delta
$$
最终得到关于横向位移 ![[公式]](https://www.zhihu.com/equation?tex=y) 和航向角 ![[公式]](https://www.zhihu.com/equation?tex=%5Cpsi) 的方程：
$$
\ddot{y} = -\frac{2C_{af} + 2C_{ar}}{mV_x}{\dot{y}} + (-V_x - \frac{2C_{af}l_f - 2C_{ar}l_r}{mV_x})\dot\psi + \frac{2C_{af}}{m}{\delta} \\
\ddot\psi = -\frac{2l_fC_{af} - 2l_rC_{ar}}{I_zV_x}\dot{y} + (-\frac{2l_f^2C_{af} + 2l_r^2C_{ar}}{I_zV_x})\dot\psi + \frac{2l_fC_{af}}{I_z}\delta
$$
设 ![[公式]](https://www.zhihu.com/equation?tex=e_1) 为横向偏差， ![[公式]](https://www.zhihu.com/equation?tex=e_2) 为航向角偏差，可以得到如下关系。
$$
\ddot{e_1} = a_y - a_{ydes} = (\ddot{y} + V_x\dot{\psi}) - V_x\dot{\psi_{des}} = \ddot{y} + V_x(\dot\psi - \dot\psi_{des}) = \ddot{y} + V_x\dot{e_2}
$$
在匀速假设下，上式可进一步推导出:
$$
\dot{e_1} =\dot{y} + V_x{e_2}
$$
因为还有，
$$
e_2 = \psi - \psi_{des}\\
\dot{e_2} = \dot{\psi} - \dot{\psi_{des}}\\
\ddot{e_2} = \ddot{\psi} - \ddot{\psi_{des}}
$$
转换为 ![[公式]](https://www.zhihu.com/equation?tex=y) 和 ![[公式]](https://www.zhihu.com/equation?tex=%5Cpsi) 的表达式，
$$
\dot{y} = \dot{e_1} - V_xe_2 \\
\ddot{y} = \ddot{e_1} - V_x\dot{e_2} \\
\dot\psi = \dot{e_2} + \dot{\psi_{des}} \\
\ddot\psi = \ddot{e_2} + \ddot{\psi_{des}}
$$
把变化后的各变量表达式代入$\ddot{y}$等式中,有如下推导过程:
$$
\ddot{y} = -\frac{2C_{af} + 2C_{ar}}{mV_x}{\dot{y}} + (-V_x - \frac{2C_{af}l_f - 2C_{ar}l_r}{mV_x})\dot\psi + \frac{2C_{af}}{m}{\delta} \\
\ddot{e_1} - V_x\dot{e_2} = -\frac{2C_{af} + 2C_{ar}}{mV_x}{(\dot{e_1} - V_x{e_2})} + (-V_x - \frac{2C_{af}l_f - 2C_{ar}l_r}{mV_x})(\dot{e_2} + \dot{\psi_{des}}) + \frac{2C_{af}}{m}{\delta} \\
\ddot{e_1} = -\frac{2C_{af} + 2C_{ar}}{mV_x}{\dot{e_1}} + \frac{2C_{af} + 2C_{ar}}{mV_x}{V_xe_2} - \frac{2C_{af}{l_f}-2C_{ar}l_r}{mV_x}{\dot{e_2}} -\frac{2C_{af}{l_f}-2C_{ar}l_r}{mV_x}{\dot{\psi_{des}}} - V_x\dot{\psi_{des}} + \frac{2C_{af}}{m}{\delta}
$$
把变化后的各变量表达式代入$\ddot{\psi}$等式中,有如下推导过程:
$$
\ddot\psi = -\frac{2l_fC_{af} - 2l_rC_{ar}}{I_zV_x}\dot{y} + (-\frac{2l_f^2C_{af} + 2l_r^2C_{ar}}{I_zV_x})\dot\psi + \frac{2l_fC_{af}}{I_z}\delta \\
\ddot\psi_{des} + \ddot{e_2} = -\frac{2l_fC_{af} - 2l_rC_{ar}}{I_zV_x}(\dot{e_1} - V_xe_2) + (-\frac{2l_f^2C_{af} + 2l_r^2C_{ar}}{I_zV_x})(\dot{e_2 + \dot{\psi_{des}}}) + \frac{2l_fC_{af}}{I_z}\delta \\
\ddot{e_2} = -\frac{2l_fC_{af} - 2l_rC_{ar}}{I_zV_x}\dot{e_1} + \frac{2l_fC_{af} - 2l_rC_{ar}}{I_zV_x}V_xe_2 - -\frac{2l_f^2C_{af} + 2l_r^2C_{ar}}{I_zV_x}\dot{e_2} - -\frac{2l_f^2C_{af} + 2l_r^2C_{ar}}{I_zV_x}{\dot{\psi_{des}}} - \ddot{\psi_{des}} + \frac{2l_fC_{af}}{I_z}\delta
$$
得到关于 ![[公式]](https://www.zhihu.com/equation?tex=e_1) 和 ![[公式]](https://www.zhihu.com/equation?tex=e_2) 的方程：
$$
\ddot{e_1} = -\frac{2C_{af} + 2C_{ar}}{mV_x}{\dot{e_1}} + \frac{2C_{af} + 2C_{ar}}{mV_x}{V_xe_2} - \frac{2C_{af}{l_f}-2C_{ar}l_r}{mV_x}{\dot{e_2}} -\frac{2C_{af}{l_f}-2C_{ar}l_r}{mV_x}{\dot{\psi_{des}}} - V_x\dot{\psi_{des}} + \frac{2C_{af}}{m}{\delta} \\
\ddot{e_2} = -\frac{2l_fC_{af} - 2l_rC_{ar}}{I_zV_x}\dot{e_1} + \frac{2l_fC_{af} - 2l_rC_{ar}}{I_zV_x}V_xe_2 - -\frac{2l_f^2C_{af} + 2l_r^2C_{ar}}{I_zV_x}\dot{e_2} - -\frac{2l_f^2C_{af} + 2l_r^2C_{ar}}{I_zV_x}{\dot{\psi_{des}}} - \ddot{\psi_{des}} + \frac{2l_fC_{af}}{I_z}\delta
$$
转换为状态空间的表达式:
$$
\frac {d}{dt}\begin{bmatrix}e_1\\ \dot{e_1}\\e_2\\ \dot{e_2} \end{bmatrix} = 
\begin{bmatrix}0 & 1 & 0 & 0\\ 0 & -{\frac{2C_{af} + 2C_{ar}}{mV_x}} &\frac{2C_{af} + 2C_{ar}}{m} & \frac{-2C_{af}{l_f} + 2C_{ar}l_r}{mV_x} \\0&0&0&1\\ 0& - \frac{2C_{af}l_f - 2C_{ar}l_r}{I_zV_x} & \frac{2C_{af}l_f - 2C_{ar}l_r}{I_z} & -\frac{2C_{af}l_f^2 + 2C_{ar}l_r^2}{I_zV_x}\end{bmatrix}\begin{bmatrix}e_1\\ \dot{e_1}\\e_2\\ \dot{e_2} \end{bmatrix} + \begin{bmatrix}0\\ \frac{2C_{af}}{m}\\0\\ \frac{2C_{af}{l_f}}{I_z} \end{bmatrix}\delta + \begin{bmatrix}0\\ 
- \frac{2C_{af} - 2C_{ar}{l_r}}{mV_x} - V_x
\\0\\ -\frac{2C_{af}l_f^2 + 2C_{ar}l_r^2}{I_zV_x} \end{bmatrix}\dot{\psi_{des}} + \begin{bmatrix}0\\ 0\\0\\ -1 \end{bmatrix}\ddot{\psi_{des}}
$$
这样就得到了在

1、小角度速度偏角

2、匀速

3、不考虑环境因素

假设下，车辆的动力学模型表达式。令
$$
A = \begin{bmatrix}0 & 1 & 0 & 0\\ 0 & -{\frac{2C_{af} + 2C_{ar}}{mV_x}} &\frac{2C_{af} + 2C_{ar}}{m} & \frac{-2C_{af}{l_f} + 2C_{ar}l_r}{mV_x} \\0&0&0&1\\ 0& - \frac{2C_{af}l_f - 2C_{ar}l_r}{I_zV_x} & \frac{2C_{af}l_f - 2C_{ar}l_r}{I_z} & -\frac{2C_{af}l_f^2 + 2C_{ar}l_r^2}{I_zV_x}\end{bmatrix} \qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\\
B = \begin{bmatrix}0\\ \frac{2C_{af}}{m}\\0\\ \frac{2C_{af}{l_f}}{I_z} \end{bmatrix} \qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\\
C = \begin{bmatrix}0\\ 
- \frac{2C_{af} - 2C_{ar}{l_r}}{mV_x} - V_x
\\0\\ -\frac{2C_{af}l_f^2 + 2C_{ar}l_r^2}{I_zV_x} \end{bmatrix}\dot{\psi_{des}} + \begin{bmatrix}0\\ 0\\0\\ -1 \end{bmatrix}\ddot{\psi_{des}} \qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\\
$$
得到状态方程，

![[公式]](https://www.zhihu.com/equation?tex=%5Cbegin%7Bequation%7D+%5Cfrac%7Bdx%7D%7Bdt%7D%3DAx%2BBu%2BC+%5Cend%7Bequation%7D) 

式中，前轮侧偏刚度 ![[公式]](https://www.zhihu.com/equation?tex=C_%7Baf%7D) 、后轮侧偏刚度 ![[公式]](https://www.zhihu.com/equation?tex=C_%7Bar%7D) 、车身质量m、转动惯量 ![[公式]](https://www.zhihu.com/equation?tex=I_z) 、前轮到重心距离 ![[公式]](https://www.zhihu.com/equation?tex=l_f) 、后轮到重心距离 ![[公式]](https://www.zhihu.com/equation?tex=l_r) 都是常量;

每个时刻的横向偏差 ![[公式]](https://www.zhihu.com/equation?tex=e_1) 、航向偏差 ![[公式]](https://www.zhihu.com/equation?tex=e_2) 、纵向速度 ![[公式]](https://www.zhihu.com/equation?tex=V_x) 、前轮转角 ![[公式]](https://www.zhihu.com/equation?tex=%5Cdelta) 都是可测量量;

目标航向 ![[公式]](https://www.zhihu.com/equation?tex=%5Cpsi) 是可获取量。

由推导出的状态方程可知，在系统中仍需要添加一个前馈控制量来消除稳态误差。

### 3.2 前馈控制量推导

在$LQR$算法中, 我们的控制系统定义为$x_{t+1} = Ax_t + Bu_t$, 对比上文推导出的车辆模型状态方程$x_{t+1} = Ax_t + Bu_t + C$, 发现缺少常数项$C$,
