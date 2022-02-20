
import numpy as np
import cv2
import imutils
from aruco_detector import AruCoDetector
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

import time

class CognitionnJudge:

    def __init__(self,marker_id,cam_mtx,dist_coeff):
        self.marker_id = marker_id
        self.cam_mtx = cam_mtx
        self.dist_coeff = dist_coeff
        self.detecter = AruCoDetector()
        self.pub = rospy.Publisher('action', Float32MultiArray, queue_size=1)
        rospy.init_node('CnJ')
        self.prex = 0
        self.prey = 0
  
    def diag2dist(self,diag):
        return (1/diag)*209.35854811

    def DecideAction(self):
        cap = cv2.VideoCapture(0)
        center = 'None'
        counting = 0
        stopNum='0'


        while cap.isOpened():
            # sec1 = time.time()
            ret,img = cap.read()
            resize_width = np.shape(img)[1]
            if ret:
                try:
                    altitude = {}
                    distance = {}


                    ids,centers,topR,bottomL,x,y,z= self.detecter.run(self.marker_id,img,self.cam_mtx,self.dist_coeff, "DICT_5X5_1000", resize_width,False)


                    if len(centers)>0:
                        for id in ids:

                            diagonal = ((topR[id][0]-bottomL[id][0])**2 + (topR[id][1]-bottomL[id][1])**2)**(0.5)
                            altitude[id] = self.diag2dist(diagonal)
                            if id == 95:
                                x[id] = x[id]+7+2*(z[id]-30)/10
                                y[id] = y[id]-6
                                z[id] = z[id]+2
                            elif id == 100:
                                z[id] = z[id]+15
                                x[id] = x[id]+2*(z[id]-30)/10
                                y[id] = y[id]-12


                            if z[id] > 300 and id == marker_id[0]:   # 고도가  300cm 이하일 경우 두번째 id만 탐지
                                # print('altitude[id] < 5 and id == marker_id[0]')
                                continue
                            elif z[id] < 300 and id == marker_id[1]:   # 고도가 300cm 이상일 경우 첫번째 id만 탐지
                                # print('altitude[id] <> 5 and id == marker_id[0]')
                                continue
                            self.prex=x[id]
                            self.prey=y[id]
                            # print(altitude[id])
                            print(",x,y ({}) : {}, {}, {}".format(id,x[id],y[id],z[id]))
                            # print
                            center = centers[id]
                            distance[id] = ((center[1]-0.5*(np.shape(img)[0]))**2+(center[0]-0.5*(np.shape(img)[1]))**2)**(0.5)


                            if abs(x[id]) < 10 and abs(y[id]) < 10:

                                print('marker detected : descend...\naction no : 0')
                                action_command = Float32MultiArray()
                                action_command.data = np.array([0,x[id],y[id],z[id]])
                                # print(action_command.data)
                                self.pub.publish(action_command)
                                stopNum = '1'
                                break

                            else:
                                print('marker detected : move to center...\naction no : 1')

                                action_command = Float32MultiArray()
                                action_command.data = np.array([1,x[id],y[id],z[id]])
                                # print(action_command.data)
                                self.pub.publish(action_command)
                                stopNum='1'
                                break

                        if stopNum =='1':
                            break

                except:
                    print('NO marker detected')
                counting += 1
                if counting > 500/3.5468: # if marker didnt detected for 5sec

                      if self.prex == 0 : 
                          # if previous center is None, then theres no marker nearby current position
                          #print('No marker detected : Landing Mission Dissabled...\naction no : -')
                          action_command = Float32MultiArray()
                          action_command.data = np.array([-1,-1,-1,-1])
                          self.pub.publish(action_command)
                          break
                      else: 
                          print('marker got out of angle : changing spot...\naction no : 2')

                          action_command = Float32MultiArray()
                          action_command.data = np.array([1,self.prex,self.prey,0])
                          self.pub.publish(action_command)
                          break
                      break

            # print(time.time()-sec1)   

        cap.release()



    def callback(self,msg):
        # print('call')
        self.DecideAction()

    def run(self):
        rospy.Subscriber("permission", String , self.callback)
        rospy.spin()



if __name__ == '__main__':
    marker_id = [95,100]
    cam_mtx = np.array([[472.98538427,   0,         384.27642545],
                        [  0,         473.94130531, 226.72986825],
                        [  0,           0,           1          ]])

    dist_coeff = np.array([[ 0.15666287, -0.36135453, -0.00808564, -0.00128795,  0.18481056]])

    CnJ = CognitionnJudge(marker_id,cam_mtx, dist_coeff)
    CnJ.run()





