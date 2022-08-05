# Infantry_Oreo 2022



本项目为北京理工大学Dream Chaser战队2022年麦轮步兵电控代码，以下为硬软件定义、电源树、通信协议与使用说明。

### 各电机ID定义

底盘电机电调配置：左前轮电调ID为1，顺时针旋转编号，反馈报文0x201-0x204。

云台PITCH轴电机反馈报文0x206（拨码[2:0]为010）；云台YAW电机反馈报文0x205（拨码[2:0]为001）。此处需在装车时提前对GM6020电机拨码进行配置。

云台拨盘2006电机反馈报文0x204。



### 各功能接口定义

##### 云台板

遥控器：UART3  (配置：波特率100000Bits/s，数据位9BITS，偶校验EVEN，停止位1)

IMU：    UART1  (配置：波特率115200Bits/s，数据位8BITS，无校验NONE，停止位1)

CAN电机通讯：FDCAN1

板间通讯：FDCAN2

SNAIL_L输出：TIM3CH1（500HZ）

SNAIL_R输出：TIM3CH2（500HZ）

ENCODER_L反馈：TIM1CH3/TIM1CH2 

ENCODER_R反馈：TIM2CH1/TIM2CH2

SNIAL上电指示：ADC2IN17

视觉通讯：USB

##### 底盘板

裁判系统通信：UART2   (配置：波特率115200Bits/s，数据位8BITS，无校验NONE，停止位1)

CAN电机通讯：FDCAN1

板间通讯：FDCAN2

电容通讯：FDCAN3



### 电源树

<img src="PowerTree.jpg" alt="PowerTree" style="zoom: 50%;" />

### 通信协议

##### 电容通信协议

待补充

##### 云台-底盘板间通信协议

待补充

##### 视觉通信协议

待补充





## 键鼠操控图谱

| 键位         | 功能                                     | 操作注释 |
| ------------ | ---------------------------------------- | -------- |
| WASD         | 车身运动控制\|能量机关 - 自瞄偏置调整1   | HOLD     |
| SHIFT+WASD   | 电容BOOST加速\|能量机关 - 自瞄偏置调整10 | HOLD     |
| Q            | 摩擦轮-开                                | DOWN     |
| CTRL+Q       | 摩擦轮-关                                | DOWN     |
| E            | 鼠标精细控制                             | HOLD     |
| R            | 切换装甲板自瞄目标                       | DOWN     |
| CTRL+R       | 切换弹仓盖状态                           | DOWN     |
| F            | 能量机关 - 自瞄偏置清零                  | DOWN     |
| Q+F          | 装甲板+哨兵 - 自瞄偏置清零               | DOWN     |
| Q+WASD       | 装甲板+哨兵 - 自瞄偏置调整1              | DOWN     |
| Q+SHIFT+WASD | 装甲板+哨兵 - 自瞄偏置调整10             | DOWN     |
| Z            | 切换电容状态                             | DOWN     |
| X            | 切换小陀螺                               | 优先级1  |
| SHIFT+X      | 切换超级小陀螺                           | 优先级0  |
| CTRL+X       | 切换屁股模式                             | 优先级2  |
| CTRL+SHIFT+X | 切换螃蟹模式                             | 优先级3  |
| SHIFT+C      | UI初始化刷新                             | DOWN     |
| B            | 切换大能量机关模式                       | DOWN     |
| V            | 切换小能量机关模式                       | DOWN     |
| G            | 切换哨兵击打模式                         | DOWN     |

