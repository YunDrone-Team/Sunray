"""
@file test_print_attitude.py
@Description: This is a test script shows how to get and print attitude data
@Author: Mohamed Abdelkader
@Contact: mohamedashraf123@gmail.com
All rights reserved 2024
"""

import sys
import os
from time import sleep
  
current = os.path.dirname(os.path.realpath(__file__))
parent_directory = os.path.dirname(current)
  
sys.path.append(parent_directory)

from siyi_sdk import SIYISDK

def test():
    cam = SIYISDK(server_ip="192.168.144.25", port=37260)
    cam.requestGimbalParam()
    sleep(1.0)
    if not cam.connect():
        print("No connection ")
        exit(1)

    i =0
    while i<10:
        print(f"帧率: {cam.getVideoFrameRateType()}")
        sleep(0.5)
        
        print(f"stream_type: {cam.getStreamType()}")
        sleep(0.5)
        print(f"video Encoding: {cam.getVideoEncType()}")
        sleep(0.5)
        print(f"Resolution Low: {cam.getResolutionLType()}")
        sleep(0.5)
        print(f"Resolution High: {cam.getResolutionHType()}")
        sleep(0.5)
        i += 1

    print('DONE')
    cam.disconnect()

if __name__ == "__main__":
    test()