Header header
int32[6] speed
###### 电机编号示意图 ######
# CAN-ID对应+1
#     |——|              |——|
#   2 |——|    front     |——| 4
#     |——|              |——|
#       |——————|  |——————|
#     0 |——————|  |——————| 1
#       |——————|  |——————|
#       |——————|  |——————|
#     |——|              |——|
#   3 |——|     back     |——| 5
#     |——|    /rear     |——|
# 速度正负：前进+ 后退- 上抬+ 下压-

int8 speed_level
#speed_level 速度档位

int8 drive_direction
#drive_direction 前进方向 1正向 0反向

int8 fin_pos_reset
#fin_pos_reset 摆臂角度重置 1重置 0正常运行

int8 fin_extend
#fin_extend 摆臂一键展开 1展开 0无动作

int8 fin_angle_mode
#摆臂角度控制 标志位 0速度控制 1角度控制(加入位置环)

float64[4] fin_expect
#摆臂角度控制 期望角度 单位：角度