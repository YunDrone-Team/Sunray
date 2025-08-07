"""
@file test_rtsp.py
@Description: Test receiving RTSP stream from IP camera
@Author: Mohamed Abdelkader
@Contact: mohamedashraf123@gmail.com
All rights reserved 2022
"""

import sys
import os
  
current = os.path.dirname(os.path.realpath(__file__))
parent_directory = os.path.dirname(current)
  
sys.path.append(parent_directory)

from stream import SIYIRTSP
from siyi_sdk import SIYISDK

def test():

    cam = SIYISDK(server_ip="192.168.144.25", port=37260)
    if not cam.connect():
        print("No connection ")
        exit(1)

    # Get camera name
    cam_str = cam.getCameraTypeString()
    cam.disconnect()
    
    rtsp = SIYIRTSP(rtsp_url="rtsp://192.168.144.25:8554/main.264",debug=False, cam_name=cam_str)
    rtsp.setShowWindow(True)
if __name__ == "__main__":
    test()