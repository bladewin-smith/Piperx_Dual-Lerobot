In this code repository, I have completed the deployment application of the Piperx Dual-Robotic-Arm under the Lerobot framework. I'm extremely grateful to the bro Yuke LIU's open-source repository:https://github.com/lykycy123/lerobot-piper.git , it has brought me tremendous help. Otherwise, it would have been very difficult for me to complete this meaningful open-source work!!!

Before run the below commands to test teleoperate dual arm, please confirm that you have the Lerobot environment(conda) and install piper_sdk 

cd Piperx_Dual-Lerobot
export LD_LIBRARY_PATH=$CONDA_PREFIX/lib:$LD_LIBRARY_PATH
python -m lerobot.scripts.control_robot --robot.type=piper --robot.inference_time=false --control.type=teleoperate

