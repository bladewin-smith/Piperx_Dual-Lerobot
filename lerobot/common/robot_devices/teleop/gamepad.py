import pygame
import threading
import time
from typing import Dict
from ros2interface import *
from piper_sdk import *
import rclpy
from rclpy.node import Node
from pyAgxArm import create_agx_arm_config, AgxArmFactory
import numpy as np
from agilex_arm_gravity_compensation.agx_pinocchio import AgxPinocchio
from scipy.spatial.transform import Rotation as R
#export PYTHONPATH=/home/jetson/lerobot-piper:$PYTHONPATH


class PiperArm(Node):
    def __init__(self, can_port: str = "can0", publish_hz: float = 200.0):
        super().__init__('piper_joint_publisher')
        self.can_port = can_port
        self.publish_hz = publish_hz
        self.joints = [0.0] * 6  # 6个关节
        self.gripper = 0.0  # 夹爪状态
        self.joint_factor = 57324.840764  # 1000*180/3.14， rad -> 度（单位0.001度）

        # 机械臂URDF路径，根据实际情况修改
        self.urdf_path = "/home/jetson/lerobot-piper/agilex_arm_gravity_compensation/piper_x_description/urdf/piper_x_description.urdf"

        # 初始化逆运动学求解器（用于重力补偿计算）
        self.pin = AgxPinocchio(self.urdf_path)

        # 控制频率
        self.control_frequency = publish_hz

        # 关节力矩修正比例（根据实际情况调整）
        self.rx_ratio = [0.25, 0.25, 0.25, 1.0, 1.0, 1.0]
        self.tx_ratio = [1.0] * 6  # 1.8-3及以上版本

        # 初始化机械臂接口
        self.cfg = create_agx_arm_config(robot="piper_x", comm="can", channel=self.can_port, interface="socketcan")
        self.robot = AgxArmFactory.create_arm(self.cfg)
        self.robot.connect()

        # 等待机械臂使能
        while not self.robot.enable():
            time.sleep(0.01)
        print("机械臂使能成功")

        # 获取当前关节角度，确保机械臂状态可用
        while self.robot.get_joint_angles() is None:
            joint_angles = np.array(self.robot.get_joint_angles().msg)
            time.sleep(0.01)
            print(joint_angles)

        # 计算世界坐标系到基座坐标系的旋转矩阵
        roll, pitch, yaw = 0, 0, 0  # 单位：deg
        self.R_world_base = R.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_matrix()

        # 初始化piper接口
        self.piper = C_PiperInterface(
            can_name=self.can_port,
            judge_flag=True,
            can_auto_init=True,
            dh_is_offset=1,
            start_sdk_joint_limit=False,
            start_sdk_gripper_limit=False,
            logger_level=LogLevel.WARNING,
            log_to_file=False,
            log_file_path=None,
        )
        self.piper.ConnectPort()
        self.piper.GripperCtrl(0, 1000, 0x01, 0)  # 使能机械臂夹爪
        time.sleep(2)
        self.piper.GripperCtrl(0, 1000, 0x00, 0)

        # 重力补偿线程控制变量
        self._gravity_thread = None
        self._gravity_thread_running = False

    def _gravity_compensation_loop(self):
        print("开始重力补偿控制循环...")
        try:
            while self._gravity_thread_running:
                start_time = time.time()

                joint_angles = np.array(self.robot.get_joint_angles().msg)

                joint_velocities = np.zeros(self.robot.joint_nums)
                joint_torques = np.zeros(self.robot.joint_nums)
                for i in range(1, self.robot.joint_nums + 1):
                    ms = self.robot.get_motor_states(i)
                    if ms is not None:
                        joint_velocities[i - 1] = ms.msg.motor_speed
                        joint_torques[i - 1] = ms.msg.torque

                gravity_torque = self.pin.gravity_compensation(joint_angles, joint_velocities, self.R_world_base)

                try:
                    for joint_id in range(1, self.robot.joint_nums + 1):
                        joint_idx = joint_id - 1
                        actual_torque = self.tx_ratio[joint_idx] * gravity_torque[joint_idx]
                        self.robot.move_mit(joint_id, 0, 0, 0, 0, actual_torque)

                        joint_torques[joint_idx] /= self.rx_ratio[joint_idx]

                    print(f"目标力矩 - 反馈力矩: {np.round(gravity_torque - joint_torques, 3).tolist()}")

                except Exception as e:
                    print(f"应用重力补偿失败: {e}")

                t = 1.0 / self.control_frequency
                elapsed_time = time.time() - start_time
                if elapsed_time < t:
                    time.sleep(t - elapsed_time)
                else:
                    print(f"警告：控制循环超时 {elapsed_time:.3f}s > {t:.3f}s")

        except Exception as e:
            print(f"重力补偿线程异常退出: {e}")

    def start_gravity_compensation(self):
        if self._gravity_thread is None or not self._gravity_thread.is_alive():
            self._gravity_thread_running = True
            self._gravity_thread = threading.Thread(target=self._gravity_compensation_loop, daemon=True)
            self._gravity_thread.start()
            print("重力补偿线程已启动")

    def stop_gravity_compensation(self):
        self._gravity_thread_running = False
        if self._gravity_thread is not None:
            self._gravity_thread.join()
            print("重力补偿线程已停止")
            self._gravity_thread = None

    def read(self) -> Dict:
        """
        获取机械臂当前关节和夹爪状态，单位0.001度
        """
        joint_msg = self.piper.GetArmJointMsgs()
        joint_state = joint_msg.joint_state

        self.joints[0] = joint_state.joint_1
        self.joints[1] = joint_state.joint_2
        self.joints[2] = joint_state.joint_3
        self.joints[3] = joint_state.joint_4
        self.joints[4] = joint_state.joint_5
        self.joints[5] = joint_state.joint_6

        gripper_msg = self.piper.GetArmGripperMsgs()
        gripper_state = gripper_msg.gripper_state
        self.gripper = gripper_state.grippers_angle

        return {
            "joint0": self.joints[0],
            "joint1": self.joints[1],
            "joint2": self.joints[2],
            "joint3": self.joints[3],
            "joint4": self.joints[4],
            "joint5": self.joints[5],
            "gripper": self.gripper
        }

    def reset(self):
        self.joints = [0.0] * 6
        self.gripper = 0.0

        joint_0 = round(self.joints[0] * self.joint_factor)
        joint_1 = round(self.joints[1] * self.joint_factor)
        joint_2 = round(self.joints[2] * self.joint_factor)
        joint_3 = round(self.joints[3] * self.joint_factor)
        joint_4 = round(self.joints[4] * self.joint_factor)
        joint_5 = round(self.joints[5] * self.joint_factor)
        gripper_range = round(self.gripper * 1000 * 1000)

        self.piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)  # joint control
        self.piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
        self.piper.GripperCtrl(abs(gripper_range), 1000, 0x01, 0)  # 单位 0.001°

    def stop(self):
        self.stop_gravity_compensation()
        self.reset()
        self.piper = None


if __name__ == "__main__":
    rclpy.init()
    arm = PiperArm(can_port="can_left_master", publish_hz=200.0)
    arm.start_gravity_compensation()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("用户中断，停止程序")
    finally:
        arm.stop()
        rclpy.shutdown()


        
# class SixAxisArmController:
#     def __init__(self):
#         # 初始化pygame和手柄
#         pygame.init()
#         pygame.joystick.init()
        
#         # 检查是否有连接的手柄
#         if pygame.joystick.get_count() == 0:
#             raise Exception("未检测到手柄")
        
#         # 初始化手柄
#         self.joystick = pygame.joystick.Joystick(0)
#         self.joystick.init()
        
#         # 初始化关节和夹爪状态
#         self.joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 6个关节
#         self.gripper = 0.0  # 夹爪状态
#         self.speeds = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 6个关节的速度
#         self.gripper_speed = 0.0  # 夹爪速度
        
#         # 定义关节弧度限制（计算好的范围）
#         self.joint_limits = [
#             (-92000 / 57324.840764, 92000 / 57324.840764),  # joint1
#             (-1300 / 57324.840764, 190000 / 57324.840764),   # joint2
#             (-80000 / 57324.840764, 0 / 57324.840764),   # joint3
#             (-90000 / 57324.840764, 90000 / 57324.840764),  # joint4
#             (-77000 / 57324.840764, 19000 / 57324.840764),  # joint5
#             (-90000 / 57324.840764, 90000 / 57324.840764)   # joint6
#         ]

#         # 启动更新线程
#         self.running = True
#         self.thread = threading.Thread(target=self.update_joints)
#         self.thread.start()
    
#     def update_joints(self):
#         while self.running:
#             # 处理事件队列
#             try:
#                 pygame.event.pump()
#             except Exception as e:
#                 self.stop()
#                 continue
                
#             # 获取摇杆和按钮输入
#             left_x = -self.joystick.get_axis(0)  # 左摇杆x轴
#             if abs(left_x) < 0.5:
#                 left_x = 0.0

#             left_y = -self.joystick.get_axis(1)  # 左摇杆y轴（取反，因为y轴向下为正
#             if abs(left_y) < 0.5:
#                 left_y = 0.0

#             right_x = -self.joystick.get_axis(2)  # 右摇杆x轴（取反，因为y轴向下为正）
#             if abs(right_x) < 0.5:
#                 right_x = 0.0
            
#             # 获取方向键输入
#             hat = self.joystick.get_hat(0)
#             up = hat[1] == 1
#             down = hat[1] == -1
#             left = hat[0] == -1
#             right = hat[0] == 1
            
#             # 获取按钮输入
#             circle = self.joystick.get_button(1)  # 圈按钮
#             cross = self.joystick.get_button(0)  # 叉按钮
#             triangle = self.joystick.get_button(4)
#             square = self.joystick.get_button(3)

#             # #debug
#             # if left:
#             #     print("Left pressed")
#             # if right:
#             #     print("Right pressed")
#             # if up:
#             #     print("Up pressed")
#             # if down:
#             #     print("Down pressed")
#             # if square: 
#             #     print("Square button pressed")
#             # if triangle:
#             #     print("Triangle button pressed")
#             # if cross:
#             #     print("Cross button pressed")
#             # if circle:
#             #     print("Circle button pressed")

            
#             # 映射输入到速度
#             self.speeds[0] = left_x * 0.01  # joint1速度
#             self.speeds[1] = left_y * 0.01  # joint2速度
#             self.speeds[2] = 0.01 if triangle else (-0.01 if square else 0.0)  # joint3速度
#             self.speeds[3] = right_x * 0.01  # joint4速度
#             self.speeds[4] = 0.01 if up else (-0.01 if down else 0.0)  # joint5速度
#             self.speeds[5] = 0.01 if right else (-0.01 if left else 0.0)  # joint6速度
#             self.gripper_speed = 0.01 if circle else (-0.01 if cross else 0.0)  # 夹爪速度
            
#             # 积分速度到关节位置
#             for i in range(6):
#                 self.joints[i] += self.speeds[i]
#             self.gripper += self.gripper_speed
            
#             # 关节范围保护
#             for i in range(6):
#                 min_val, max_val = self.joint_limits[i]
#                 self.joints[i] = max(min_val, min(max_val, self.joints[i]))
            
#             # 夹爪范围保护（0~0.08弧度）
#             self.gripper = max(0.0, min(0.08, self.gripper))
            
#             # 控制更新频率
#             time.sleep(0.02)
    
#     def get_action(self) -> Dict:
#         # 返回机械臂的当前状态
#         return {
#             'joint0': self.joints[0],
#             'joint1': self.joints[1],
#             'joint2': self.joints[2],
#             'joint3': self.joints[3],
#             'joint4': self.joints[4],
#             'joint5': self.joints[5],
#             'gripper': self.gripper
#         }
    
#     def stop(self):
#         # 停止更新线程
#         self.running = False
#         self.thread.join()
#         pygame.quit()
#         print("Gamepad exits")

#     def reset(self):
#         self.joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 6个关节
#         self.gripper = 0.0  # 夹爪状态
#         self.speeds = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 6个关节的速度
#         self.gripper_speed = 0.0  # 夹爪速度

# # 使用示例
# if __name__ == "__main__":
#     arm_controller = SixAxisArmController()
#     try:
#         while True:
#             print(arm_controller.get_action())
#             time.sleep(0.1)
#     except KeyboardInterrupt:
#         arm_controller.stop()
