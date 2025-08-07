"""
@file test_set_angles.py
@Description: 循环交互设置 SIYI 云台 yaw 和 pitch 角度，直到用户主动退出

"""

import sys
import os
from time import sleep
  
current = os.path.dirname(os.path.realpath(__file__))
parent_directory = os.path.dirname(current)
  
sys.path.append(parent_directory)

from siyi_sdk import SIYISDK

def main():
    cam = SIYISDK(server_ip="192.168.144.25", port=37260)

    if not cam.connect():
        print("连接失败 ")
        exit(1)
    cam.requestHardwareID() # Important to get the angles limits defined in cameras.py
    sleep(1)


    print(" 云台连接成功，输入 yaw 和 pitch 角度控制云台，输入 q 退出。\n")

    while True:
        try:
            yaw_input = input("请输入目标 yaw 角度（-135~135度),输入 q 退出：")
            if yaw_input.lower() in ['q', 'quit', 'exit']:
                break
            pitch_input = input("请输入目标 pitch 角度（-90~25度),输入 q 退出：")
            if pitch_input.lower() in ['q', 'quit', 'exit']:
                break
            
            target_yaw_deg = float(yaw_input)
            target_pitch_deg = float(pitch_input)
            
            cam.requestSetAngles(target_yaw_deg, target_pitch_deg)
            sleep(1)
        
            print("当前姿态 (yaw,pitch,roll) :", cam.getAttitude())

        except ValueError:
            print("输入格式错误，请输入数字")
        except KeyboardInterrupt:
            print("手动中断，准备退出...")
            break

    print("操作完成，退出..")
    cam.disconnect()

if __name__ == "__main__":
    main()