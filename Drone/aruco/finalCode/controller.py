#!/usr/bin/env python3
import asyncio
from turtle import position
import cv2
from numpy.core.numeric import True_
import rospy
import numpy as np
import sys


from mavsdk import System
from mavsdk.offboard import (AccelerationNed, OffboardError, VelocityNedYaw, Attitude)
from mavsdk.offboard import (OffboardError, PositionNedYaw)

from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

# Drone
class Controller():
    
    def __init__(self):
        self.EmergencyCode = 0
        self.drone = System()
        self.is_in_air = False
        self.mission = None
        self.loop = asyncio.get_event_loop()
        rospy.init_node('controller')
        self.pub = rospy.Publisher('isDone', String,queue_size=5)
        self.landingpub = rospy.Publisher('permission', String,queue_size=5)
        self.actiondata = [-10,-1,-1,-1]


    async def connect(self):
        self.drone = System()
        drone = self.drone
        # await drone.connect(system_address="serial:///dev/ttyUSB0:921600")
        await self.drone.connect(system_address="udp://:14540")
        print('px4 drone connected')


        # print("-- Waiting for drone to connect...")
        # async for state in self.drone.core.connection_state():
            # if state.is_connected:
                # print(f"Drone discovered with UUID: {state.uuid}")
                # break

        print("-- Setting initial setpoint")
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
        await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0, 0))

        print("-- Starting offboard")
        try:
            await self.drone.offboard.start()
        except OffboardError as error:
            print(f"Starting offboard mode failed with error code: \
                {error._result.result}")
            print("-- Disarming")
            await self.drone.action.disarm()
            return





    async def takeoff(self):
        print("-- Arming")
        # await self.emersub()
        await self.drone.action.arm()
        await asyncio.sleep(3)
        print("-- Go Up 0.5 m/s")
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, -1, 0.0))
        # await self.drone.action.takeoff()
        await asyncio.sleep(10)
        print("-- Mission Done")
        self.pub.publish('mission done')
        # await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.1, 0.0))
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        await self.drone.offboard.set_acceleration_ned(AccelerationNed(0,0,0))
        await asyncio.sleep(2)





    async def land(self):

        print("landing mission start")
        self.landingpub.publish('mission done')
        rospy.Subscriber('action', Float32MultiArray, self.ActionDecider)
        while True:
            
            if self.actiondata[0] == -1:
                print("Landing mission disabled : NO MARKER DETECTED")
                # self.landingpub.publish('mission done')
                self.pub.publish('mission done NO MARKER DETECTED')
                # await asyncio.sleep(0.01)
                # await self.drone.action.return_to_launch()
                self.actiondata[0] = 10
                break

            elif self.actiondata[0] == 1:
                print("Approach to marker")
                # print(self.actiondata)
                # await asyncio.sleep(0.01)
                await self.PositionAdjusting(self.actiondata[1],self.actiondata[2],0)
                self.actiondata[0] = 10


            elif self.actiondata[0] == 0:
                if self.actiondata[3] > 0 and self.actiondata[3] < 45 : # cm or m
                    await self.land2()
                    self.pub.publish('mission done')
                    break
                else:
                    print("descending")
                    # print(self.actiondata)
                    await self.PositionAdjusting(0,0,self.actiondata[3])
                    self.actiondata[0] = 10


        
            await asyncio.sleep(0.01)

                
    async def PositionAdjusting(self,x,y,z):
        n = -y/100
        e = x/100
        d = 0.1
        if z==0:
            d=0
        elif z>1:
            d = z/1000
        elif 0<z<1:
            d = 0.1
            
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(n, e, d, 0.0))
        print("-- Moving -- velocity : [{:.2f},{:.2f},{:.2f}], time : 1s".format(n,e,d))
        await asyncio.sleep(1)
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0.0, 0.0))
        # await asyncio.sleep(0.5)
        self.landingpub.publish('mission done')



    # async def Down(self):
    #     print("-- Landing")
    #     await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0,0.0,0.2,0.0))
    #     await asyncio.sleep(2)
    #     print("-- Landing finished")
    #     await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0,0.0,0.0,0.0))
    #     self.pub.publish('mission done')
    #     # await asyncio.sleep(2)

    async def move(self):
        # try :
        print("-- move")
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(5.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(3)
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(4.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(3)
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(4.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(3)
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(4.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(3)
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        self.pub.publish('mission done')
        await asyncio.sleep(1)

        




    async def run(self):
        print('run')
        await self.connect()
        # await asyncio.sleep(1)
        print('-- ready for mission')
        while True:
            
            if self.mission == 'takeoff':
                print(f'mission : {self.mission}')
                self.mission = None
                # await asyncio.sleep(0.01)
                # print(f'mission : {self.mission}')
                await self.takeoff()
                        
            elif self.mission == 'land':
                print(f'mission : {self.mission}')
                self.mission = None
                # await asyncio.sleep(0.01)
                # print(f'mission : {self.mission}')
                await self.land()

            elif self.mission == 'move':
                print(f'mission : {self.mission}')
                self.mission = None
                # await asyncio.sleep(0.01)
                # print(f'mission : {self.mission}')
                await self.move()


            print(self.is_in_air)
            await asyncio.sleep(0.1)




    def MissionStarter(self,msg):

        self.mission = msg.data

    def ActionDecider(self,msg):

        self.actiondata = np.asarray(msg.data)

    


    async def sub(self):

        print('subscriber On')
        rospy.Subscriber('mission', String, self.MissionStarter)



    async def observe_is_in_air(self):
        """ Monitors whether the drone is flying or not and
        returns after landing """
        await asyncio.sleep(10)
        print('is onAir observing start')
        async for is_in_air in self.drone.telemetry.in_air():
            self.is_in_air = is_in_air
            await asyncio.sleep(0.01)

        # if data.data== 'q':
        #     await self.drone.offboard.set_acceleration_ned(AccelerationNed(0,0,0))
        # await asyncio.sleep(0.5)

    async def EmergencyStop(self):
            while True:
                if self.EmergencyCode ==1:

                    await self.drone.action.land()
                    print('emerency landing')
                    self.EmergencyCode = 0
                await asyncio.sleep(0.1)

    async def Emergencysub(self):
        print('subscriber On')
        rospy.Subscriber('emer_code', String, self.Emergency_callback)
        

    def Emergency_callback(self,msg):
        self.EmergencyCode = 1
        print('EMER')

                




if __name__ == '__main__':
    c = Controller()

    c.loop.run_until_complete(asyncio.gather(c.run(),c.sub(),c.observe_is_in_air(),c.EmergencyStop(),c.Emergencysub()))