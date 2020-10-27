#!/usr/bin/env python
import rospy
#from audio_common_msgs.msg import AudioData
from hark_msgs.msg import HarkWave

rospy.init_node("hoge")
def callback(msg):
    lth = len(msg.src[0].wavedata)
    print(lth)
rospy.Subscriber("/HarkWave", HarkWave, callback)
rospy.spin()
