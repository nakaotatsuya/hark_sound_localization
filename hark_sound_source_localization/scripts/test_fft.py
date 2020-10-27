#!/usr/bin/env python
import rospy
#from audio_common_msgs.msg import AudioData
from hark_msgs.msg import HarkWave, HarkFFT

rospy.init_node("hoge")
def callback(msg):
    lth = len(msg.src[0].fftdata_imag)
    print(lth)
rospy.Subscriber("/HarkFFT", HarkFFT, callback)
rospy.spin()

