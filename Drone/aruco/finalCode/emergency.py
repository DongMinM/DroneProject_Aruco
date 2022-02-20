
#!/usr/bin/env python3

import rospy # import python ros pakage which was for c++
from std_msgs.msg import String # in ros, structure "Message" is used. among standard messages, import String
import numpy as np


def talker():
    rospy.init_node('Emer_code', anonymous=True) # this python code will be a node
    while True:
        input('INPUT EMERGENCY CODE (ANY KEY) : ')
        pub = rospy.Publisher('emer_code', String, queue_size=1) #queue size : time how much you will keep
        msg = 'EMERGENCY'
        pub.publish(msg)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass








