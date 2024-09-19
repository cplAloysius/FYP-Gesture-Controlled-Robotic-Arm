#!/usr/bin/env python
# coding: utf-8

import Arm_Lib
from time import sleep


class identify_grap:
    def __init__(self):
        # set move status
        # 设置移动状态
        self.move_status = True
        self.arm = Arm_Lib.Arm_Device()
        # Clamping jaw tightening angle
        # 夹爪加紧角度
        self.grap_joint = 135
        self.holding = 0

    def pickup(self, joints):
        '''
        Moving process
        移动过程
        :param joints: 移动到物体位置的各关节角度   The angle of each joint moved to the position of the object
        :param joints_up: 机械臂抬起各关节角度     The manipulator raises the angle of each joint
        :param color_angle: 移动到对应垃圾桶的角度  Move to the angle of the corresponding trash can
        '''
        joints_uu = [joints[0], 80, 50, 50, 265, self.grap_joint]
        # raise 抬起
        #joints_up = [joints_down[0], 80, 50, 50, 265, 30]
        # put up 架起
        self.arm.Arm_serial_servo_write6_array(joints_uu, 1500)
        sleep(1.5)
        # Opening and closing jaws 开合夹爪
        for i in range(5):
            self.arm.Arm_serial_servo_write(6, 180, 100)
            sleep(0.08)
            self.arm.Arm_serial_servo_write(6, 30, 100)
            sleep(0.08)
        # Release clamping jaws 松开夹爪
        # self.arm.Arm_serial_servo_write(6, 0, 500)
        # sleep(0.5)
        # Move to object position 移动至物体位置
        self.arm.Arm_serial_servo_write6_array(joints, 500)
        sleep(0.5)
        # Grasp and clamp the clamping claw进行抓取,夹紧夹爪
        self.arm.Arm_serial_servo_write(6, self.grap_joint, 500)
        sleep(0.5)
        # put up 架起
        self.arm.Arm_serial_servo_write6_array(joints_uu, 1000)
        sleep(1)
        self.holding = 1

        # #put down
        # self.arm.Arm_serial_servo_write6_array(joints_down, 1000)
        # sleep(1)

        # # Lift to above the corresponding position 抬起至对应位置上方
        # self.arm.Arm_serial_servo_write(1, joints_down[0], 500)
        # sleep(0.5)
        # # Move to target location 移动至目标位置
        # self.arm.Arm_serial_servo_write6_array(joints_down, 1000)
        # sleep(1)
        
        # Release the object and release the clamping jaws释放物体,松开夹爪
        # self.arm.Arm_serial_servo_write(6, 30, 500)
        # sleep(0.5)
        # # raise  抬起
        # self.arm.Arm_serial_servo_write6_array(joints_uu, 1000)
        # sleep(1)
        
    def putdown(self, joints_down):
        joints_uu = [joints_down[0], 80, 50, 50, 265, self.grap_joint]
        #put down
        self.arm.Arm_serial_servo_write6_array(joints_down, 1000)
        sleep(1)
        # Release the object and release the clamping jaws释放物体,松开夹爪
        self.arm.Arm_serial_servo_write(6, 30, 500)
        sleep(0.5)
        # raise  抬起
        self.arm.Arm_serial_servo_write6_array(joints_uu, 1000)
        sleep(1)
        self.holding = 0

    def identify_move(self, joints):
        '''
        Manipulator movement function 机械臂移动函数
        :param name:识别的颜色  Recognized color
        :param joints: 反解求得的各关节角度 Angle of each joint obtained by inverse solution
        '''
        if self.move_status == True:
            self.move_status = False
            #joints_down = [45, 50, 20, 60, 265, self.grap_joint]
            if not self.holding:
                joints = [joints[0], joints[1], joints[2], joints[3], 265, 30]
                self.pickup(joints)
            else:
                if joints[0] == None:
                    joints[0] = self.arm.Arm_serial_servo_read(1)
                joints = [joints[0], joints[1], joints[2], joints[3], 265, self.grap_joint]
                self.putdown(joints)
                
            self.move_status = True
            return self.holding
            
        return -1
            
#         if name == "red" and self.move_status == True:
#             # It is set here. You can only run down after this operation
#             # 此处设置,需执行完本次操作,才能向下运行
#             self.move_status = False
#             # print ("red")
#             # print (joints[0], joints[1], joints[2], joints[3], joints[4])
#             # Obtain the target joint angle 获得目标关节角
#             joints = [joints[0], joints[1], joints[2], joints[3], 265, 30]
#             # Attitude before moving to the target 移动到目标前的姿态
# #             joints_down = [45, 80, 35, 40, 265, self.grap_joint]
#             joints_down = [45, 50, 20, 60, 265, self.grap_joint]
#             self.move(joints, joints_down)
#             self.move_status = True
#         if name == "blue" and self.move_status == True:
#             self.move_status = False
#             # print ("blue")
#             # print (joints[0], joints[1], joints[2], joints[3], joints[4])
#             joints = [joints[0], joints[1], joints[2], joints[3], 265, 30]
# #             joints_down = [27, 110, 0, 40, 265, self.grap_joint]
#             joints_down = [27, 75, 0, 50, 265, self.grap_joint]
#             self.move(joints, joints_down)
#             self.move_status = True
#         if name == "green" and self.move_status == True:
#             self.move_status = False
#             # print ("green")
#             # print (joints[0], joints[1], joints[2], joints[3], joints[4])
#             joints = [joints[0], joints[1], joints[2], joints[3], 265, 30]
# #             joints_down = [152, 110, 0, 40, 265, self.grap_joint]
#             joints_down = [152, 75, 0, 50, 265, self.grap_joint]
#             self.move(joints, joints_down)
#             self.move_status = True
#         if name == "yellow" and self.move_status == True:
#             self.move_status = False
#             # print ("yellow")
#             # print (joints[0], joints[1], joints[2], joints[3], joints[4])
#             joints = [joints[0], joints[1], joints[2], joints[3], 265, 30]
# #             joints_down = [137, 80, 35, 40, 265, self.grap_joint]
#             joints_down = [137, 50, 20, 60, 265, self.grap_joint]
#             self.move(joints, joints_down)
#             self.move_status = True