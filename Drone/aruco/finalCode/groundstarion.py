
#!/usr/bin/env python3

import cv2
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import asyncio


class GroundStation:
    def __init__(self):
        self.pub = rospy.Publisher('mission', String, queue_size = 10)
        rospy.init_node('GroundStation', anonymous=False)
        self.loop = asyncio.get_event_loop()
        self.mission = 'mission done'
    


    async def run(self):

        while True:
            if self.mission == 'mission done' or self.mission == 'mission done NO MARKER DETECTED':
                mission = input('input the mission : ')
                print()
                if mission == 'takeoff':
                    self.mission = 'Doing'
                    print('Mission 1 : Take Off recieved')
                    self.pub.publish(mission)
                    print('takeoff')

                elif mission == 'land':
                    self.mission = 'Doing'
                    print('Mission 2 : Land recieved')
                    self.pub.publish(mission)

                elif mission == 'land!':
                    self.mission = 'Doing'
                    print('Mission 3 : emergency landing')
                    self.pub.publish(mission)


                else:
                    print('Unvaliable mission')
                    continue
            else :
                continue
            await asyncio.sleep(0.1)

    def callback(self,msg):
        print('Mission Done!!')
        self.mission = msg.data
        if msg.data == 'mission done NO MARKER DETECTED':
            print('NO MARKER DETECTED')



    async def sub(self):
        print('waiting for mission done....')
        rospy.Subscriber("isDone", String, self.callback)


if __name__ == "__main__":
    G = GroundStation()
    G.loop.run_until_complete(asyncio.gather(G.run(),G.sub()))




