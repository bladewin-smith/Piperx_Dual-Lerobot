Before run the below commands to test teleoperate dual arm, please confirm that you have the Lerobot environment(conda) and install piper_sdk 



# Piperx Dual-Robotic-Arm 部署与使用说明（基于Lerobot框架）

感谢开源大神 **Yuke LIU** 的仓库 [lerobot-piper](https://github.com/lykycy123/lerobot-piper.git) 提供了宝贵资源，助力本项目顺利完成！

---

## 目录

- [环境准备](#环境准备)
- [依赖安装](#依赖安装)
- [项目运行](#项目运行)
- [功能测试](#功能测试)
- [常见问题及解决方案](#常见问题及解决方案)
- [联系方式](#联系方式)

---

## 环境准备

1. **安装Miniconda或Anaconda**

   - 推荐使用 [Miniconda](https://docs.conda.io/en/latest/miniconda.html) 轻量级Python环境管理工具。
   - 安装完成后，确保 `conda` 命令可用。

2. **创建并激活Lerobot虚拟环境**

   ```bash
   conda create -n lerobot python=3.10 -y
   conda activate lerobot
   cd Piperx_Dual-Lerobot
   export LD_LIBRARY_PATH=$CONDA_PREFIX/lib:$LD_LIBRARY_PATH
   python -m lerobot.scripts.control_robot --robot.type=piper --robot.inference_time=false --control.type=teleoperate
