#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import os
import numpy as np
import rospy
import rospkg

current_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_dir)

from tf_interface.tf_interface import TFInterface

class Test(object):
    
    def __init__(self):
        
        # ========= TF interface for visualization =========
        self.tf_interface = TFInterface()
        rospy.loginfo('TF Interface Ready')
        
    
if __name__ == '__main__':
    rospy.init_node('Drag_control')
    server = Test()
    rate = rospy.Rate(10)  

    try:
        pass
    except KeyboardInterrupt:
        print("KeyboardInterrupt occur!")
