#!/usr/bin/env python3
#-*- coding: utf-8 -*-
# by bailiang，2022.03.08
# 记得修改.py文件可执行权限！！！

import rospy
import time
from nubot_msgs.msg import base_drive_cmd
from nubot_msgs.msg import base_info
from nubot_msgs.msg import link_msg
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JoyFeedbackArray
from sensor_msgs.msg import JoyFeedback
from geometry_msgs.msg import Pose
# import tf

# import pickle
import numpy as np

msg = """
Cheat Sheet for Nubot !
---------------------------
Moving around: (left stick) ↑ ↓ ← →
Moving Fins:
    L1↑ L2↓      R1↑ R2↓

   Delta↑ X↓    Delta↑ X↓
---------------------------
Share/Option : increase/decrease motor speeds
anything else : force stop

CTRL-C to quit
"""

# class Control_2D_Real(object):
#     def __init__(self):
#         self.fin_plan = [0,0,0,0]
#         try:
#             # load plan from file
#             with open("/home/nubot-1941/workspace/2D-Real/src/nubot_teleop/scripts/plan_realexp.pickle", "rb") as f:# 
#                 self.P_x, self.MinPath, self.MinCost = pickle.load(f)
#             self.plan_isvalid = True
#         except FileNotFoundError:
#             self.plan_isvalid = False
#             print("plan is not valid")
#         try:
#             with open("/home/nubot-1941/workspace/2D-Real/src/nubot_teleop/scripts/direct_realexp_result.pickle", "rb") as f:#phi_h_touch_FR
#                 self.phi_map, self.h_map, self.touch_map, self.time_map, self.theta_B, self.theta_F  = pickle.load(f)
#         except FileNotFoundError:
#             print("predict result is not valid")

#         self.fillP(self.P_x, self.MinPath)

#     def fillP(self, P_x, MinPath):# 插补规划路径点之间的动作 fill the motion between plan point
#         # P_x = self.predictor.P_x
#         C_x = P_x
#         # C_x = np.delete(C_x,[0,1,-2,-1])
#         C_x_1 = C_x
#         num_insert = 10 # 插补数量          

#         C_b = np.rad2deg(self.theta_B[list(map(int,MinPath[:,0]))]) # back/left flipper motion
#         C_f = np.rad2deg(self.theta_F[list(map(int,MinPath[:,1]))]) # front/right flipper motion
#         C_b_1 = C_b
#         C_f_1 = C_f
#         t = 0
#         for i in range(0,len(C_x)-1):
#             if (num_insert >= 2):
#                 x_delta = (C_x[i+1] - C_x[i]) / (num_insert)
#                 l_delta = (C_b[i+1] - C_b[i]) / (num_insert)
#                 r_delta = (C_f[i+1] - C_f[i]) / (num_insert)
#                 for j in range(1,int(num_insert)):
#                     C_x_1 = np.insert(C_x_1, i+t+j, C_x[i]+x_delta*(j))
#                     C_b_1 = np.insert(C_b_1, i+t+j, C_b[i]+l_delta*(j))
#                     C_f_1 = np.insert(C_f_1, i+t+j, C_f[i]+r_delta*(j))
#                 t = t+num_insert-1
#         self.C_x, self.C_b, self.C_f = C_x_1, C_b_1, C_f_1

#     def control(self,robot_position):
#         ### place your control code here
#         robot_position = robot_position * 250 #单位转换
#         ind_x = np.searchsorted(self.C_x, robot_position) #to match the plt_map 
#         if self.plan_isvalid:
#             if ind_x <0:
#                 ind_x = 0
#             if ind_x > len(self.C_x)-1:
#                 ind_x = len(self.C_x)-1
#             self.fin_plan[0] = self.C_f[ind_x] # front fin
#             self.fin_plan[2] = self.C_f[ind_x] # front fin
#             self.fin_plan[1] = self.C_b[ind_x] # front fin
#             self.fin_plan[3] = self.C_b[ind_x] # front fin
#         return self.fin_plan

class Callbacks:
    def __init__(self):
        self.speed = 200
        self.speed_fin = 600     #摆臂速度by chenghao 2019.3.13
        self.speed_gears = [200, 600, 1000, 1500, 2000, 2500, 3000]
        self.gears = 0
        self.velocity = [0,0,0,0,0,0]
        self.drive_direction = 1 #前进方向 1正向 0反向
        self.drive_direction_auto = -1
        self.joy_emcy = 0

        self.fin_pos_reset = 0
        self.fin_angle_mode = 0
        self.fin_auto_mode = 0
        self.fin_add = [0,0,0,0]
        self.fin_add_delta = 0.5
        self.fin_expect_pub = [0,0,0,0]
        self.fin_angle_real = [0,0,0,0]
        self.fin_angle_auto = [0,0,0,0]
        self.driver_emcy = 0
        # self.vive_set_zero = 0
        # self.vive_pos_count = 0
        # self.vive_pos_add = 0
        # self.vive_pos_bias = 0

        self.flag_dir = 0
        self.flag_fin_reset = 0
        self.flag_angle_mode = 0
        self.flag_gear_down = 0
        self.flag_gear_up = 0
        
        # self.flag_set_zero = 0
        self.flag_joy_emcy = 0
        self.flag_auto_mode = 0
        self.now_control = 0

        self.flag_joy_feedback = 0

        # self.base_matrix = np.identity(4)
        # self.vive_matrix = np.eye(4,4)
        # self.vive_base_matrix = np.identity(4)
        # self.vive_base_matrix[:,3] = [-0.16175, 0.0, -0.138, 1]

        # self.control_alg = Control_2D_Real()
        
    def link_callback(self, link_msg):
        self.now_control = link_msg.control_flag

    def base_info_callback(self, base_info):
        self.driver_emcy = base_info.emcy
        self.fin_angle_real[0] = base_info.fin_angle[0]
        self.fin_angle_real[1] = base_info.fin_angle[1]
        self.fin_angle_real[2] = base_info.fin_angle[2]
        self.fin_angle_real[3] = base_info.fin_angle[3]

    def fin_auto_callback(self, fin_auto_msg):
        self.fin_angle_auto[0] = fin_auto_msg.fin_expect[0]
        self.fin_angle_auto[1] = fin_auto_msg.fin_expect[1]
        self.fin_angle_auto[2] = fin_auto_msg.fin_expect[2]
        self.fin_angle_auto[3] = fin_auto_msg.fin_expect[3]
        self.drive_direction_auto = fin_auto_msg.drive_direction
    
    # def vive_set_zero_fun(self):
    #     if self.vive_pos_count < 20:
    #         self.vive_pos_add += self.base_matrix[0][3]
    #         self.vive_pos_count += 1
    #     else:
    #         self.vive_pos_bias = self.vive_pos_add/20
    #         self.vive_pos_count = 0
    #         self.vive_pos_add = 0
    #         self.vive_set_zero = 0
    #         print('vive_pos_bias:',self.vive_pos_bias)

    def joycallback(self, joy):
        if self.now_control==0:
            ###### 倒车模式 ######
            ###by bailiang 2022.04.08 更改按键
            if joy.axes[6] != self.flag_dir: #判断按键是否和上一循环一样，避免重复触发
                if joy.axes[6] == 1: #按键按下，倒车模式
                    self.flag_joy_feedback = 1
                    self.drive_direction = -1
                    if self.fin_angle_mode == 1:
                        self.fin_expect_pub[0] = self.fin_angle_real[3]
                        self.fin_expect_pub[1] = self.fin_angle_real[2]
                        self.fin_expect_pub[2] = self.fin_angle_real[1]
                        self.fin_expect_pub[3] = self.fin_angle_real[0]
                    self.flag_dir = 1 #按键按下，状态置1。记忆本次循环按键状态
                elif joy.axes[6] == -1: #按键按下，正常模式
                    self.flag_joy_feedback = 1
                    self.drive_direction = 1
                    if self.fin_angle_mode == 1:
                        self.fin_expect_pub[0] = self.fin_angle_real[0]
                        self.fin_expect_pub[1] = self.fin_angle_real[1]
                        self.fin_expect_pub[2] = self.fin_angle_real[2]
                        self.fin_expect_pub[3] = self.fin_angle_real[3]
                    self.flag_dir = -1 #按键按下，状态置-1。记忆本次循环按键状态
                    # if self.drive_direction == 0: #改变前进方向
                    #     self.drive_direction = 1
                    # else:
                    #     self.drive_direction = 0
                    # self.flag_dir = 1 #按键按下，状态置1。记忆本次循环按键状态
                else:
                    self.flag_joy_feedback = 0
                    self.flag_dir = 0 #按键松开，状态归零

            ###### 摆臂位置重置 ######
            ###by bailiang 2022.03.08
            if joy.buttons[11] != self.flag_fin_reset: #判断按键是否和上一循环一样，避免重复触发
                if joy.buttons[11] == 1: #按键按下
                    self.flag_joy_feedback = 1
                    self.fin_pos_reset = 1
                    self.fin_expect_pub[0] = 0
                    self.fin_expect_pub[1] = 0
                    self.fin_expect_pub[2] = 0
                    self.fin_expect_pub[3] = 0
                    self.flag_fin_reset = 1 #按键按下，状态置1。记忆本次循环按键状态
                else:
                    self.flag_joy_feedback = 0
                    self.flag_fin_reset = 0 #按键松开，状态归零

            ###### 机器人遥控急停 ######
            ###by bailiang 2022.09.13
            if joy.buttons[1] != self.flag_joy_emcy: #判断按键是否和上一循环一样，避免重复触发
                if joy.buttons[1] == 1: #按键按下
                    self.flag_joy_feedback = 1
                    if self.joy_emcy == 0: #开始置零位
                        self.joy_emcy = 1
                    else:
                        self.joy_emcy = 0
                    self.flag_joy_emcy = 1 #按键按下，状态置1。记忆本次循环按键状态
                else:
                    self.flag_joy_feedback = 0
                    self.flag_joy_emcy = 0 #按键松开，状态归零

            ###### 摆臂角度控制模式 ######
            ###by bailiang 2022.04.08 
            if joy.buttons[10] != 0:
                if joy.buttons[12] != self.flag_angle_mode: #判断按键是否和上一循环一样，避免重复触发
                    if joy.buttons[12] == 1: #按键按下
                        self.flag_joy_feedback = 1
                        if self.fin_angle_mode == 0: #改变摆臂角度控制模式
                            self.fin_angle_mode = 1
                            if self.drive_direction == 1:
                                self.fin_expect_pub[0] = self.fin_angle_real[0]
                                self.fin_expect_pub[1] = self.fin_angle_real[1]
                                self.fin_expect_pub[2] = self.fin_angle_real[2]
                                self.fin_expect_pub[3] = self.fin_angle_real[3]
                            else:
                                self.fin_expect_pub[0] = self.fin_angle_real[3]
                                self.fin_expect_pub[1] = self.fin_angle_real[2]
                                self.fin_expect_pub[2] = self.fin_angle_real[1]
                                self.fin_expect_pub[3] = self.fin_angle_real[0]
                        else:
                            self.fin_angle_mode = 0
                        self.flag_angle_mode = 1 #按键按下，状态置1。记忆本次循环按键状态
                    else:
                        self.flag_joy_feedback = 0
                        self.flag_angle_mode = 0 #按键松开，状态归零

                ###### 摆臂角度自动控制模式 ######
                ###by bailiang 2022.04.09
                if joy.buttons[3] != self.flag_auto_mode: #判断按键是否和上一循环一样，避免重复触发
                    if joy.buttons[3] == 1: #按键按下
                        self.flag_joy_feedback = 1
                        if self.fin_auto_mode == 0: #改变摆臂控制模式
                            self.fin_auto_mode = 1
                            self.drive_direction = self.drive_direction_auto
                        else:
                            self.fin_auto_mode = 0
                            self.drive_direction = self.drive_direction_auto
                        self.flag_auto_mode = 1 #按键按下，状态置1。记忆本次循环按键状态
                    else:
                        self.flag_joy_feedback = 0
                        self.flag_auto_mode = 0 #按键松开，状态归零

                ###### 机器人归零位 ######
                ###by bailiang 2022.04.09
                # if joy.buttons[1] != self.flag_set_zero: #判断按键是否和上一循环一样，避免重复触发
                #     if joy.buttons[1] == 1: #按键按下
                #         if self.vive_set_zero == 0: #开始置零位
                #             self.vive_set_zero = 1
                #         # else:
                #         #     self.set_zero = 0
                #         self.flag_set_zero = 1 #按键按下，状态置1。记忆本次循环按键状态
                #     else:
                #         self.flag_set_zero = 0 #按键松开，状态归零

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
            # 手柄正负：左摇杆：前axes1+ 后axes1- 左axes0+ 右axes0-
            ##### 主履带和摆臂by bailaing 2022.04.08 ######
            #主履带一直是速度控制模式
            linear = abs(joy.axes[1])
            turn = abs(joy.axes[0])
            #直线行进by chenghao 2019.3.13
            if linear >= turn:
                self.velocity[0] = round(joy.axes[1])*self.speed #根据手柄输入决定正负
                self.velocity[1] = round(joy.axes[1])*self.speed
            #转弯by chenghao 2019.3.13
            else:
                if self.gears >= 2 :
                    self.velocity[0] = -round(joy.axes[0])*1000    #round(joy.axes[0])*self.speed
                    self.velocity[1] = round(joy.axes[0])*1000    #-round(joy.axes[0])*self.speed
                else :
                    self.velocity[0] = -round(joy.axes[0])*self.speed    #round(joy.axes[0])*self.speed
                    self.velocity[1] = round(joy.axes[0])*self.speed    #-round(joy.axes[0])*self.speed

            #摆臂速度控制模式
            if self.fin_auto_mode == 0:
                if self.fin_angle_mode == 0: 
                    #front left
                    if joy.buttons[4] == 1: #ps4手柄 L1
                        self.velocity[2] = self.speed_fin #上抬
                        # self.velocity[4] = self.speed_fin #上抬
                    elif joy.axes[2] == -1: #ps4手柄 L2
                        self.velocity[2] = -self.speed_fin #下压
                        # self.velocity[4] = -self.speed_fin #下压
                    else:
                        self.velocity[2] = 0
                        # self.velocity[4] = 0

                    #front right
                    if joy.buttons[5] == 1: #ps4手柄 R1
                        self.velocity[4] = self.speed_fin #上抬
                    elif joy.axes[5] == -1: #ps4手柄 R2
                        self.velocity[4] = -self.speed_fin #下压
                    else:
                        self.velocity[4] = 0

                    #rear 后摆臂共同控制
                    if joy.buttons[2] == 1: #ps4手柄 三角
                        self.velocity[3] = self.speed_fin #上抬
                        self.velocity[5] = self.speed_fin #上抬
                    elif joy.buttons[0] == 1: #ps4手柄 叉
                        self.velocity[3] = -self.speed_fin #下压
                        self.velocity[5] = -self.speed_fin #下压
                    else:
                        self.velocity[3] = 0
                        self.velocity[5] = 0

                    #rear right 后摆臂分开控制
                    #rear left
                    # if joy.buttons[3] == 1: #ps4手柄 方框
                    #     self.velocity[3] = self.speed_fin #上抬
                    # elif joy.buttons[0] ==  1: #ps4手柄 叉
                    #     self.velocity[3] = -self.speed_fin #下压
                    # else:
                    #     self.velocity[3] = 0
                    # #rear right
                    # if joy.buttons[2] == 1: #ps4手柄 三角
                    #     self.velocity[5] = self.speed_fin #上抬
                    # elif joy.buttons[1] ==  1: #ps4手柄 圈
                    #     self.velocity[5] = -self.speed_fin #下压
                    # else:
                    #     self.velocity[5] = 0
                #摆臂角度控制模式
                else:
                    #front left
                    if joy.buttons[4] == 1: #ps4手柄 L1
                        self.fin_add[0] = self.fin_add_delta #上抬
                    elif joy.axes[2] == -1: #ps4手柄 L2
                        self.fin_add[0] = -self.fin_add_delta #下压
                    else:
                        self.fin_add[0] = 0

                    #front right
                    if joy.buttons[5] == 1: #ps4手柄 R1
                        self.fin_add[2] = self.fin_add_delta #上抬
                    elif joy.axes[5] == -1: #ps4手柄 R2
                        self.fin_add[2] = -self.fin_add_delta #下压
                    else:
                        self.fin_add[2] = 0

                    #rear 后摆臂共同控制
                    if joy.buttons[2] == 1: #ps4手柄 三角
                        self.fin_add[1] = self.fin_add_delta #上抬
                        self.fin_add[3] = self.fin_add_delta #上抬
                    elif joy.buttons[0] == 1: #ps4手柄 叉
                        self.fin_add[1] = -self.fin_add_delta #下压
                        self.fin_add[3] = -self.fin_add_delta #下压
                    else:
                        self.fin_add[1] = 0
                        self.fin_add[3] = 0
                    
            ###### 调速by chenghao 2019.3.13 ######
            if joy.buttons[8] != self.flag_gear_down:
                if joy.buttons[8] == 1:
                    #by chenghao 2019.3.13
                    self.gears -= 1
                    if self.gears < 0:
                        self.gears = 0
                    self.speed = self.speed_gears[self.gears]
                    # print (self.speed)
                    self.flag_gear_down = 1
                else:
                    self.flag_gear_down = 0

            if joy.buttons[9] != self.flag_gear_up:
                if joy.buttons[9] == 1:
                    #by chenghao 2019.3.13
                    self.gears += 1
                    if self.gears > len(self.speed_gears)-1:
                        self.gears = len(self.speed_gears)-1
                    self.speed = self.speed_gears[self.gears]
                    # print (self.speed)
                    self.flag_gear_up = 1
                else:
                    self.flag_gear_up = 0

    def timercallback(self, event):		
        cmd = base_drive_cmd()
        cmd.speed = self.velocity
        if self.fin_auto_mode == 0:
            cmd.drive_direction = self.drive_direction
        else:
            cmd.drive_direction = self.drive_direction_auto
        cmd.speed_level = self.gears + 1
        cmd.fin_pos_reset = self.fin_pos_reset
        self.fin_pos_reset = 0 #摆臂复位按键，发送一次之后置0，防止重复触发

        # if self.vive_set_zero == 1:
        #     self.vive_set_zero_fun()

        if self.driver_emcy == 1 or self.joy_emcy == 1: #急停开关拍下后，强制转为摆臂速度控制模式,并将期望角度置0
            self.fin_angle_mode = 0
            self.fin_auto_mode = 0
            self.fin_expect_pub[0] = 0
            self.fin_expect_pub[1] = 0
            self.fin_expect_pub[2] = 0
            self.fin_expect_pub[3] = 0
        if self.fin_angle_mode == 1:
            if self.fin_auto_mode == 1:
                # self.fin_expect_pub = self.control_alg.control(self.robot_position)
                self.fin_expect_pub[0] = self.fin_angle_auto[0]
                self.fin_expect_pub[1] = self.fin_angle_auto[1]
                self.fin_expect_pub[2] = self.fin_angle_auto[2]
                self.fin_expect_pub[3] = self.fin_angle_auto[3]
                # print('robot_position:',self.robot_position)
                # print('auto_fin_expect:',self.fin_expect_pub)
            else:
                self.fin_expect_pub[0] += self.fin_add[0]
                self.fin_expect_pub[1] += self.fin_add[1]
                self.fin_expect_pub[2] += self.fin_add[2]
                self.fin_expect_pub[3] += self.fin_add[3]
        cmd.fin_angle_mode = self.fin_angle_mode
        cmd.fin_expect = self.fin_expect_pub
        cmd.emcy = self.joy_emcy
        cmd_pub.publish(cmd)
        # print('velocity gears:',self.velocity,self.gears)

    def timercallback_1(self, event):		
        joy_back_msg = JoyFeedbackArray()
        rumble_msg = JoyFeedback()
        rumble_msg.type = 1
        rumble_msg.id = 0
        if self.flag_joy_feedback == 1:
            rumble_msg.intensity = 1.0
        else:
            rumble_msg.intensity = 0
        joy_back_msg.array.append(rumble_msg)
        joy_feedback_pub.publish(joy_back_msg)
        

if __name__=='__main__':
    callbacks = Callbacks()
    rospy.init_node('nubot_teleop',anonymous=True)

    joy_sub = rospy.Subscriber('/joy',Joy,callbacks.joycallback)
    base_info_sub = rospy.Subscriber('/nubot_drive/base_info',base_info,callbacks.base_info_callback)
    fin_auto_sub = rospy.Subscriber('/nubot_drive/fin_auto_cmd',base_drive_cmd,callbacks.fin_auto_callback) 
    link_sub = rospy.Subscriber('/control_flag',link_msg,callbacks.link_callback)

    cmd_pub = rospy.Publisher('/nubot_drive/base_drive_cmd', base_drive_cmd, queue_size=10)
    joy_feedback_pub = rospy.Publisher('/joy/set_feedback', JoyFeedbackArray, queue_size=10)

    timer = rospy.Timer(period = rospy.Duration(0.02), callback = callbacks.timercallback) #50Hz
    timer_1 = rospy.Timer(period = rospy.Duration(0.005), callback = callbacks.timercallback_1) #200Hz

    # print (msg)
    rospy.spin()

