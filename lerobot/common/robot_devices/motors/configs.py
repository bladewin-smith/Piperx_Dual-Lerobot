import abc
from dataclasses import dataclass

import draccus


#抽象基类 `MotorsBusConfig`
#继承自 `MotorsBusConfig`，并通过 `draccus.ChoiceRegistry` 实现了子类注册机制
@dataclass
class MotorsBusConfig(draccus.ChoiceRegistry, abc.ABC):
    @property
    def type(self) -> str:  #通过 `.type` 属性获取注册时的字符串标识
        return self.get_choice_name(self.__class__)


#三个具体的电机总线配置子类：`DynamixelMotorsBusConfig`、`FeetechMotorsBusConfig` 和 `PiperMotorsBusConfig`
#每个子类都用 `@MotorsBusConfig.register_subclass("name")` 装饰器注册
#方便通过字符串 `"dynamixel"`、`"feetech"`、`"piper"` 来识别和实例化对应配置
#每个配置类都定义了总线端口名称和电机字典（`motors`），其中字典的键是电机名称
# 值是一个元组 `(int, str)`，表示电机ID和型号
#`mock` 参数用于是否模拟模式（仅部分子类有）。
@MotorsBusConfig.register_subclass("dynamixel")
@dataclass
class DynamixelMotorsBusConfig(MotorsBusConfig):
    port: str
    motors: dict[str, tuple[int, str]]
    mock: bool = False


@MotorsBusConfig.register_subclass("feetech")
@dataclass
class FeetechMotorsBusConfig(MotorsBusConfig):
    port: str
    motors: dict[str, tuple[int, str]]
    mock: bool = False


@MotorsBusConfig.register_subclass("piper")
@dataclass
class PiperMotorsBusConfig(MotorsBusConfig):
    can_name: str
    motors: dict[str, tuple[int, str]]

