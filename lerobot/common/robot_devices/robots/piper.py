"""
    Teleoperation Agilex Piper with a Piper-X leader-arm or a PS5 controller    
"""

import time
import torch
import numpy as np
from dataclasses import dataclass, field, replace
import rclpy

from lerobot.common.robot_devices.teleop.gamepad import  PiperArm #SixAxisArmController,
from lerobot.common.robot_devices.motors.utils import get_motor_names, make_motors_buses_from_configs
from lerobot.common.robot_devices.cameras.utils import make_cameras_from_configs
from lerobot.common.robot_devices.utils import RobotDeviceAlreadyConnectedError, RobotDeviceNotConnectedError
from lerobot.common.robot_devices.robots.configs import PiperRobotConfig
#以下的代码主要用于控制主臂
class PiperRobot:
    def __init__(self, config: PiperRobotConfig | None = None, **kwargs):
        if config is None:
            config = PiperRobotConfig()
        # Overwrite config arguments using kwargs
        self.config = replace(config, **kwargs)
        self.robot_type = self.config.type
        self.inference_time = self.config.inference_time # if it is inference time
        
        # 初始化rclpy
        if not rclpy.ok():
            rclpy.init()
        # build cameras
        self.cameras = make_cameras_from_configs(self.config.cameras)
        
        # build piper motors
        self.piper_motors_left_slave = make_motors_buses_from_configs(self.config.follower_arm_left)
        self.piper_motors_right_slave = make_motors_buses_from_configs(self.config.follower_arm_right)
        # self.arml = self.piper_motorsm['main']
        self.arm_left_slave = self.piper_motors_left_slave['slave']
        self.arm_right_slave = self.piper_motors_right_slave['slave']
        self.can_port_left_leader = self.config.leader_arm_left["main"].can_name
        self.can_port_right_leader = self.config.leader_arm_right["main"].can_name
        self.can_port_left_slave = self.config.follower_arm_left["slave"].can_name
        self.can_port_right_slave = self.config.follower_arm_right["slave"].can_name
        
        # build gamepad teleop
        if not self.inference_time:
            self.teleop_left_leader = PiperArm(can_port=self.can_port_left_leader, publish_hz=200.0)
            self.teleop_left_leader.piper.GripperCtrl(0,1000,0x01,0)
            time.sleep(2)
            self.teleop_left_leader.piper.GripperCtrl(0,1000,0x00,0)
            self.teleop_right_leader = PiperArm(can_port=self.can_port_right_leader, publish_hz=200.0)
            self.teleop_right_leader.piper.GripperCtrl(0,100,0x01,0)
            time.sleep(2)
            self.teleop_right_leader.piper.GripperCtrl(0,100,0x00,0)
            # self.teleop = SixAxisArmController()
        else:
            self.teleop_left_leader = None
            self.teleop_right_leader = None
        self.teleop_left_leader.piper.GripperCtrl(0,1000,0x00,0)
        self.teleop_right_leader.piper.GripperCtrl(0,100,0x00,0)
        
        self.logs = {}
        self.is_connected = False

    @property
    def camera_features(self) -> dict:
        cam_ft = {}
        for cam_key, cam in self.cameras.items():
            key = f"observation.images.{cam_key}"
            cam_ft[key] = {
                "shape": (cam.height, cam.width, cam.channels),
                "names": ["height", "width", "channels"],
                "info": None,   
            }
        return cam_ft

    
    @property
    def motor_features(self) -> dict:
        action_names_left_slave = get_motor_names(self.piper_motors_left_slave)
        state_names_left_slave = get_motor_names(self.piper_motors_left_slave)
        action_names_right_slave = get_motor_names(self.piper_motors_right_slave)
        state_names_right_slave = get_motor_names(self.piper_motors_right_slave)
        return {
            "action": {
                "dtype": "float32",
                "shape_left": (len(action_names_left_slave),),
                "names_left": action_names_left_slave,
                "shape_right": (len(action_names_right_slave),),
                "names_right": action_names_right_slave,
            },
            "observation.state": {
                "dtype": "float32",
                "shape_left": (len(state_names_left_slave),),
                "names_left": state_names_left_slave,
                "shape_right": (len(state_names_right_slave),),
                "names_right": state_names_right_slave,
            },
        }
    
    @property
    def has_camera(self):
        return len(self.cameras) > 0

    @property
    def num_cameras(self):
        return len(self.cameras)


    def connect(self) -> None:
        """Connect piper and cameras"""
        if self.is_connected:
            raise RobotDeviceAlreadyConnectedError(
                "Piper is already connected. Do not run `robot.connect()` twice."
            )
        
        # connect piper
        self.arm_left_slave.connect(enable=True)
        self.arm_right_slave.connect(enable=True)
        self.teleop_left_leader.piper.GripperCtrl(0,1000,0x00,0)
        self.teleop_right_leader.piper.GripperCtrl(0,100,0x00,0)
        print("piper conneted")

        # connect cameras
        for name in self.cameras:
            self.cameras[name].connect()
            self.is_connected = self.is_connected and self.cameras[name].is_connected
            print(f"camera {name} conneted")
        
        print("All connected")
        self.is_connected = True
        
        self.run_calibration()


    def disconnect(self) -> None:
        """move to home position, disenable piper and cameras"""
        # move piper to home position, disable
        if not self.inference_time:
            self.teleop_left_leader.stop()
            self.teleop_right_leader.stop()

        # disconnect piper
        self.arm_left_slave.safe_disconnect()
        self.arm_right_slave.safe_disconnect()
        print("piper disable after 5 seconds")
        time.sleep(5)
        self.arm_left_slave.connect(enable=False)
        self.arm_right_slave.connect(enable=False)

        # disconnect cameras
        if len(self.cameras) > 0:
            for cam in self.cameras.values():
                cam.disconnect()

        self.is_connected = False


    def run_calibration(self):
        """move piper to the home position"""
        if not self.is_connected:
            raise ConnectionError()
        
        self.arm_left_slave.apply_calibration()
        self.teleop_left_leader.piper.GripperCtrl(0,1000,0x00,0)
        self.teleop_right_leader.piper.GripperCtrl(0,100,0x00,0)
        if not self.inference_time:
            self.teleop_left_leader.reset()
            self.teleop_right_leader.reset()



    def teleop_step(
        self, record_data=False
    ) -> None | tuple[dict[str, torch.Tensor], dict[str, torch.Tensor]]:  #定义遥操机械臂实现步骤的方法
        self.teleop_left_leader.piper.GripperCtrl(0,1000,0x00,0)
        self.teleop_right_leader.piper.GripperCtrl(0,100,0x00,0)
        if not self.is_connected:
            raise ConnectionError()
        
        if self.teleop_left_leader is None and self.inference_time:
            self.teleop_left_leader = PiperArm(can_port=self.can_port_left_leader, publish_hz=50.0)
            self.teleop_left_leader.piper.GripperCtrl(0,1000,0x00,0)
            self.teleop_right_leader = PiperArm(can_port=self.can_port_right_leader, publish_hz=200.0)
            self.teleop_right_leader.piper.GripperCtrl(0,100,0x00,0)
            # self.teleop = SixAxisArmController()
        #
        # read target pose state as 
        before_read_t = time.perf_counter()
        state_left = self.arm_left_slave.read() # read current joint position from robot
        action_left = self.teleop_left_leader.read()
        state_right = self.arm_right_slave.read() # read current joint position from robot
        action_right = self.teleop_right_leader.read()
        # action = self.teleop.get_action() # target joint position from gamepad
        self.logs["read_pos_dt_s"] = time.perf_counter() - before_read_t

        # do action
        before_write_t = time.perf_counter()
        target_joints_left = list(action_left.values())
        self.arm_left_slave.write(target_joints_left)
        target_joints_right = list(action_right.values())
        self.arm_right_slave.write(target_joints_right)
        self.logs["write_pos_dt_s"] = time.perf_counter() - before_write_t
        self.teleop_left_leader.start_gravity_compensation()
        self.teleop_right_leader.start_gravity_compensation()
        if not record_data:
            return
        
        state_left = torch.as_tensor(list(state_left.values()), dtype=torch.float32)
        action_left = torch.as_tensor(list(action_left.values()), dtype=torch.float32)
        state_right = torch.as_tensor(list(state_right.values()), dtype=torch.float32)
        action_right = torch.as_tensor(list(action_right.values()), dtype=torch.float32)

        # Capture images from cameras
        images = {}
        for name in self.cameras:
            before_camread_t = time.perf_counter()
            images[name] = self.cameras[name].async_read()
            images[name] = torch.from_numpy(images[name])
            self.logs[f"read_camera_{name}_dt_s"] = self.cameras[name].logs["delta_timestamp_s"]
            self.logs[f"async_read_camera_{name}_dt_s"] = time.perf_counter() - before_camread_t

        # Populate output dictionnaries
        obs_dict, action_dict = {}, {}
        obs_dict["observation.state.left"] = state_left
        obs_dict["observation.state.right"] = state_right

        action_dict["action.left"] = action_left
        action_dict["action.right"] = action_right

        for name in self.cameras:
            obs_dict[f"observation.images.{name}"] = images[name]

        return obs_dict, action_dict



    def send_action(self, action: torch.Tensor) -> torch.Tensor:
        """Write the predicted actions from policy to the motors"""
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                "Piper is not connected. You need to run `robot.connect()`."
            )

        # send to motors, torch to list
        target_joints = action.tolist()
        self.arms.write(target_joints)

        return action



    def capture_observation(self) -> dict:
        """capture current images and joint positions"""
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                "Piper is not connected. You need to run `robot.connect()`."
            )
        
        # read current joint positions
        before_read_t = time.perf_counter()
        state = self.arms.read()  # 6 joints + 1 gripper
        self.logs["read_pos_dt_s"] = time.perf_counter() - before_read_t

        state = torch.as_tensor(list(state.values()), dtype=torch.float32)

        # read images from cameras
        images = {}
        for name in self.cameras:
            before_camread_t = time.perf_counter()
            images[name] = self.cameras[name].async_read()
            images[name] = torch.from_numpy(images[name])
            self.logs[f"read_camera_{name}_dt_s"] = self.cameras[name].logs["delta_timestamp_s"]
            self.logs[f"async_read_camera_{name}_dt_s"] = time.perf_counter() - before_camread_t

        # Populate output dictionnaries and format to pytorch
        obs_dict = {}
        obs_dict["observation.state"] = state
        for name in self.cameras:
            obs_dict[f"observation.images.{name}"] = images[name]
        return obs_dict
    
    def teleop_safety_stop(self):
        """ move to home position after record one episode """
        self.run_calibration()

    
    def __del__(self):
        if self.is_connected:
            self.disconnect()
            if not self.inference_time:
                self.teleop.stop()
                
