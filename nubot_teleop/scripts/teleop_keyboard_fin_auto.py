#!/usr/bin/env python3

from __future__ import print_function

import threading

import roslib; roslib.load_manifest('nubot_teleop')
import rospy
from nubot_msgs.msg import base_info, base_drive_cmd

import sys, select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

msg = """
Control robot using keyboard
---------------------------
Moving around:
        w     
   a    s    d
        x
---------------------------
q/e : decrease/increase speeds
z/c : change moving direction
---------------------------
Flipper control:
   u ↑ front o ↑
   j ↓       l ↓
        i ↑
        k ↓
       back
---------------------------
t : flipper-auto mode!!!
m : flipper angle/speed mode switch

CTRL-C to quit
"""

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        base_info_sub = rospy.Subscriber('/nubot_drive/base_info',base_info,self.base_info_callback)
        fin_auto_sub = rospy.Subscriber('/nubot_drive/fin_auto_cmd',base_drive_cmd,self.fin_auto_callback) 
        self.publisher = rospy.Publisher('/nubot_drive/base_drive_cmd', base_drive_cmd, queue_size = 1)

        self.speed = 200
        self.speed_fin = 600     #摆臂速度by chenghao 2019.3.13
        self.speed_gears = [200, 400, 600, 1000, 1500, 2000, 2500, 3000]
        self.gears = 0
        self.velocity = [0,0,0,0,0,0]
        self.drive_direction = 1 #前进方向 1正向 0反向
        self.drive_direction_auto = -1

        self.fin_pos_reset = 0
        self.fin_angle_mode = 1 #Gazebo摆臂速度控制模式不太好用，直接角度控制
        self.fin_auto_mode = 0
        self.fin_add = [0,0,0,0]
        self.fin_add_delta = 0.75 #摆臂转速调整
        self.fin_expect_pub = [0,0,0,0]
        self.fin_angle_real = [0,0,0,0]
        self.fin_angle_auto = [0,0,0,0]
        self.driver_emcy = 0

        self.flag_dir = 0
        self.flag_fin_reset = 0
        self.flag_angle_mode = 0
        self.flag_gear_down = 0
        self.flag_gear_up = 0

        self.flag_joy_emcy = 0
        self.flag_auto_mode = 0

        self.settings = self.saveTerminalSettings()
        self.condition = threading.Condition()
        self.done = False
        
        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    # def update(self, x, y, z, th, speed, turn):
    #     self.condition.acquire()
    #     self.x = x
    #     self.y = y
    #     self.z = z
    #     self.th = th
    #     self.speed = speed
    #     self.turn = turn
    #     # Notify publish thread that we have a new message.
    #     self.condition.notify()
    #     self.condition.release()

    def stop(self):
        self.done = True
        # self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        cmd = base_drive_cmd()
        while not self.done:
            cmd.speed = self.velocity
            if self.fin_auto_mode == 0:
                cmd.drive_direction = self.drive_direction
            else:
                cmd.drive_direction = self.drive_direction_auto
            cmd.speed_level = self.gears + 1
            cmd.fin_pos_reset = self.fin_pos_reset
            self.fin_pos_reset = 0 #摆臂复位按键，发送一次之后置0，防止重复触发

            if self.driver_emcy == 1: #急停开关拍下后，强制转为摆臂速度控制模式,并将期望角度置0
                self.fin_angle_mode = 0
                self.fin_auto_mode = 0
                self.fin_expect_pub[0] = 0
                self.fin_expect_pub[1] = 0
                self.fin_expect_pub[2] = 0
                self.fin_expect_pub[3] = 0
            if self.fin_angle_mode == 1:
                if self.fin_auto_mode == 1:
                    self.fin_expect_pub[0] = self.fin_angle_auto[0]
                    self.fin_expect_pub[1] = self.fin_angle_auto[1]
                    self.fin_expect_pub[2] = self.fin_angle_auto[2]
                    self.fin_expect_pub[3] = self.fin_angle_auto[3]
                else:
                    self.fin_expect_pub[0] += self.fin_add[0]
                    self.fin_expect_pub[1] += self.fin_add[1]
                    self.fin_expect_pub[2] += self.fin_add[2]
                    self.fin_expect_pub[3] += self.fin_add[3]
                    self.fin_add[0] = 0
                    self.fin_add[1] = 0
                    self.fin_add[2] = 0
                    self.fin_add[3] = 0
            cmd.fin_angle_mode = self.fin_angle_mode
            cmd.fin_expect = self.fin_expect_pub
            self.publisher.publish(cmd)
            rospy.sleep(rospy.Duration(self.timeout))


# class Callbacks:
#     def __init__(self):
        # self.speed = 200
        # self.speed_fin = 600     #摆臂速度by chenghao 2019.3.13
        # self.speed_gears = [200, 400, 600, 1000, 1500, 2000, 2500, 3000]
        # self.gears = 0
        # self.velocity = [0,0,0,0,0,0]
        # self.drive_direction = 1 #前进方向 1正向 0反向
        # self.drive_direction_auto = -1

        # self.fin_pos_reset = 0
        # self.fin_angle_mode = 1 #Gazebo摆臂速度控制模式不太好用，直接角度控制
        # self.fin_auto_mode = 0
        # self.fin_add = [0,0,0,0]
        # self.fin_add_delta = 0.8 #摆臂转速调整
        # self.fin_expect_pub = [0,0,0,0]
        # self.fin_angle_real = [0,0,0,0]
        # self.fin_angle_auto = [0,0,0,0]
        # self.driver_emcy = 0

        # self.flag_dir = 0
        # self.flag_fin_reset = 0
        # self.flag_angle_mode = 0
        # self.flag_gear_down = 0
        # self.flag_gear_up = 0

        # self.flag_joy_emcy = 0
        # self.flag_auto_mode = 0

        # self.settings = self.saveTerminalSettings()

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

    def getKey(self,settings):
        if sys.platform == 'win32':
            # getwch() returns a string on Windows
            key = msvcrt.getwch()
        else:
            tty.setraw(sys.stdin.fileno())
            # sys.stdin.read() returns a string on Linux
            key = sys.stdin.read(1)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def saveTerminalSettings(self):
        if sys.platform == 'win32':
            return None
        return termios.tcgetattr(sys.stdin)

    def restoreTerminalSettings(self):
        if sys.platform == 'win32':
            return
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def keyboard_callback(self,key):
        ###### 倒车模式 ######
        # try:
            
            print("keyboard:",key)
            print(msg)

            # if self.drive_direction == 1:
            #     self.fin_expect_pub[0] = self.fin_angle_real[0]
            #     self.fin_expect_pub[1] = self.fin_angle_real[1]
            #     self.fin_expect_pub[2] = self.fin_angle_real[2]
            #     self.fin_expect_pub[3] = self.fin_angle_real[3]
            # else:
            #     self.fin_expect_pub[0] = self.fin_angle_real[3]
            #     self.fin_expect_pub[1] = self.fin_angle_real[2]
            #     self.fin_expect_pub[2] = self.fin_angle_real[1]
            #     self.fin_expect_pub[3] = self.fin_angle_real[0]

            ###by bailiang 2022.04.08 更改按键
            if key == 'z': #按键按下，倒车模式
                if self.flag_dir == 0: #判断按键是否和上一循环一样，避免重复触发
                    self.drive_direction = 0
                    if self.fin_angle_mode == 1:
                        self.fin_expect_pub[0] = self.fin_angle_real[3]
                        self.fin_expect_pub[1] = self.fin_angle_real[2]
                        self.fin_expect_pub[2] = self.fin_angle_real[1]
                        self.fin_expect_pub[3] = self.fin_angle_real[0]
                    self.flag_dir = 1 #按键按下，状态置1。记忆本次循环按键状态
            else:
                self.flag_dir = 0 #按键松开，状态归零

            if key == 'c': #按键按下，正常模式
                if self.flag_dir == 0: 
                    self.drive_direction = 1
                    if self.fin_angle_mode == 1:
                        self.fin_expect_pub[0] = self.fin_angle_real[0]
                        self.fin_expect_pub[1] = self.fin_angle_real[1]
                        self.fin_expect_pub[2] = self.fin_angle_real[2]
                        self.fin_expect_pub[3] = self.fin_angle_real[3]
                    self.flag_dir = 1 #按键按下，状态置1。记忆本次循环按键状态
            else:
                self.flag_dir = 0 #按键松开，状态归零

            # ###### 摆臂位置重置 ######
            # ###by bailiang 2022.03.08
            # if joy.buttons[11] != flag_fin_reset: #判断按键是否和上一循环一样，避免重复触发
            #     if joy.buttons[11] == 1: #按键按下
            #         fin_pos_reset = 1
            #         fin_expect_pub[0] = 0
            #         fin_expect_pub[1] = 0
            #         fin_expect_pub[2] = 0
            #         fin_expect_pub[3] = 0
            #         flag_fin_reset = 1 #按键按下，状态置1。记忆本次循环按键状态
            #     else:
            #         flag_fin_reset = 0 #按键松开，状态归零

            ###### 摆臂角度控制模式 ######
            ##by bailiang 2022.04.08 
            if key == 'm': #判断按键是否和上一循环一样，避免重复触发
                if self.flag_angle_mode == 0: #按键按下
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
                self.flag_angle_mode = 0 #按键松开，状态归零

            ###### 摆臂角度自动控制模式 ######
            ###by bailiang 2022.04.09
            if key == 't': #判断按键是否和上一循环一样，避免重复触发
                if self.flag_auto_mode == 0:
                    if self.fin_auto_mode == 0: #改变摆臂控制模式
                        self.fin_auto_mode = 1
                        self.drive_direction = self.drive_direction_auto
                    else:
                        self.fin_auto_mode = 0
                        self.drive_direction = self.drive_direction_auto
                    self.flag_auto_mode = 1 #按键按下，状态置1。记忆本次循环按键状态
            else:
                self.flag_auto_mode = 0 #按键松开，状态归零

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
            #直线行进
            if key == 'w': #按键按下，前进
                self.velocity[0] = self.speed
                self.velocity[1] = self.speed
            elif key == 'x': #按键按下，后退
                self.velocity[0] = -1*self.speed
                self.velocity[1] = -1*self.speed
            #转弯
            elif key == 'a':
                if self.gears >= 2 :
                    self.velocity[0] = -1000
                    self.velocity[1] =  1000
                else:
                    self.velocity[0] = -1*self.speed
                    self.velocity[1] =    self.speed
            elif key == 'd':
                if self.gears >= 2 :
                    self.velocity[0] =  1000
                    self.velocity[1] = -1000
                else:
                    self.velocity[0] =    self.speed
                    self.velocity[1] = -1*self.speed
            elif key == 's':
                self.velocity[0] = 0
                self.velocity[1] = 0
            else:
                self.velocity[0] = 0
                self.velocity[1] = 0

            #摆臂速度控制模式
            if self.fin_auto_mode == 0:
                if self.fin_angle_mode == 0: 
                    #front left
                    if key == 'u': #键盘 u
                        self.velocity[2] = self.speed_fin #上抬
                    elif 'j': #键盘 j
                        self.velocity[2] = -self.speed_fin #下压
                    else:
                        self.velocity[2] = 0

                    #front right
                    if key == 'o': #键盘 o
                        self.velocity[4] = self.speed_fin #上抬
                    elif key == 'l': #键盘 l
                        self.velocity[4] = -self.speed_fin #下压
                    else:
                        self.velocity[4] = 0

                    #rear 后摆臂共同控制
                    if key == 'i': #键盘 u
                        self.velocity[3] = self.speed_fin #上抬
                        self.velocity[5] = self.speed_fin #上抬
                    elif key == 'k': #键盘 u
                        self.velocity[3] = -self.speed_fin #下压
                        self.velocity[5] = -self.speed_fin #下压
                    else:
                        self.velocity[3] = 0
                        self.velocity[5] = 0

                    # #rear right 后摆臂分开控制
                    # #rear left
                    # if joy.buttons[3] == 1: #ps4手柄 方框
                    #     velocity[3] = speed_fin #上抬
                    # elif joy.buttons[0] ==  1: #ps4手柄 叉
                    #     velocity[3] = -speed_fin #下压
                    # else:
                    #     velocity[3] = 0
                    # #rear right
                    # if joy.buttons[2] == 1: #ps4手柄 三角
                    #     velocity[5] = speed_fin #上抬
                    # elif joy.buttons[1] ==  1: #ps4手柄 圈
                    #     velocity[5] = -speed_fin #下压
                    # else:
                    #     velocity[5] = 0
                #摆臂角度控制模式
                else:
                    #front left
                    if key == 'u': #键盘 u
                        self.fin_add[0] = self.fin_add_delta #上抬
                    elif key == 'j': #键盘 j
                        self.fin_add[0] = -self.fin_add_delta #下压
                    else:
                        self.fin_add[0] = 0

                    #front right
                    if key == 'o': #键盘 o
                        self.fin_add[2] = self.fin_add_delta #上抬
                    elif key == 'l': #键盘 l
                        self.fin_add[2] = -self.fin_add_delta #下压
                    else:
                        self.fin_add[2] = 0

                    #rear 后摆臂共同控制
                    if key == 'i': #键盘 i
                        self.fin_add[1] = self.fin_add_delta #上抬
                        self.fin_add[3] = self.fin_add_delta #上抬
                    elif key == 'k': #键盘 k
                        self.fin_add[1] = -self.fin_add_delta #下压
                        self.fin_add[3] = -self.fin_add_delta #下压
                    else:
                        self.fin_add[1] = 0
                        self.fin_add[3] = 0
                    
            ###### 调速by chenghao 2019.3.13 ######
            if key == 'q': 
                if self.flag_gear_down == 0:
                    #by chenghao 2019.3.13
                    self.gears -= 1
                    if self.gears < 0:
                        self.gears = 0
                    self.speed = self.speed_gears[self.gears]
                    print (self.speed)
                    self.flag_gear_down = 1
            else:
                self.flag_gear_down = 0

            if key == 'e':
                if self.flag_gear_up == 0:
                    #by chenghao 2019.3.13
                    self.gears += 1
                    if self.gears > len(self.speed_gears)-1:
                        self.gears = len(self.speed_gears)-1
                    self.speed = self.speed_gears[self.gears]
                    print (self.speed)
                    self.flag_gear_up = 1
            else:
                self.flag_gear_up = 0

            # if key == '\x03':
            #     rospy.signal_shutdown("CTRL-C")

        # except Exception as e:
        #     print(e)


    # def timercallback(self, event):	
    #     print(msg)
    #     callbacks.keyboard_callback()
    #     cmd = base_drive_cmd()
    #     cmd.speed = self.velocity
    #     if self.fin_auto_mode == 0:
    #         cmd.drive_direction = self.drive_direction
    #     else:
    #         cmd.drive_direction = self.drive_direction_auto
    #     cmd.speed_level = self.gears + 1
    #     cmd.fin_pos_reset = self.fin_pos_reset
    #     self.fin_pos_reset = 0 #摆臂复位按键，发送一次之后置0，防止重复触发

    #     if self.driver_emcy == 1: #急停开关拍下后，强制转为摆臂速度控制模式,并将期望角度置0
    #         self.fin_angle_mode = 0
    #         self.fin_auto_mode = 0
    #         self.fin_expect_pub[0] = 0
    #         self.fin_expect_pub[1] = 0
    #         self.fin_expect_pub[2] = 0
    #         self.fin_expect_pub[3] = 0
    #     if self.fin_angle_mode == 1:
    #         if self.fin_auto_mode == 1:
    #             self.fin_expect_pub[0] = self.fin_angle_auto[0]
    #             self.fin_expect_pub[1] = self.fin_angle_auto[1]
    #             self.fin_expect_pub[2] = self.fin_angle_auto[2]
    #             self.fin_expect_pub[3] = self.fin_angle_auto[3]
    #         else:
    #             self.fin_expect_pub[0] += self.fin_add[0]
    #             self.fin_expect_pub[1] += self.fin_add[1]
    #             self.fin_expect_pub[2] += self.fin_add[2]
    #             self.fin_expect_pub[3] += self.fin_add[3]
    #     cmd.fin_angle_mode = self.fin_angle_mode
    #     cmd.fin_expect = self.fin_expect_pub
    #     cmd_pub.publish(cmd)

if __name__=="__main__":
    # callbacks = Callbacks()
    rospy.init_node('teleop_keyboard',anonymous=True)

    # base_info_sub = rospy.Subscriber('/nubot_drive/base_info',base_info,callbacks.base_info_callback)
    # fin_auto_sub = rospy.Subscriber('/nubot_drive/fin_auto_cmd',base_drive_cmd,callbacks.fin_auto_callback) 

    # cmd_pub = rospy.Publisher('/nubot_drive/base_drive_cmd', base_drive_cmd, queue_size = 10)

    # timer = rospy.Timer(period = rospy.Duration(0.02), callback = callbacks.timercallback) #50Hz

    # rospy.spin()
    # callbacks.restoreTerminalSettings()
    
    pub_thread = PublishThread(50)
    try:
        pub_thread.wait_for_subscribers()
        # pub_thread.update(x, y, z, th, speed, turn)

        while(1):
            key = pub_thread.getKey(pub_thread.settings)
            pub_thread.keyboard_callback(key)
            if key == '\x03':
                break
            # pub_thread.update(x, y, z, th, speed, turn)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        pub_thread.restoreTerminalSettings(pub_thread.settings)