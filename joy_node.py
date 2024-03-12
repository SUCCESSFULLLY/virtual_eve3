#!/usr/bin/env python

import rospy, os
import pandas as pd

import numpy as np #행렬 생성
import math

from morai_msgs.msg import CtrlCmd, EventInfo, Lamps
from sensor_msgs.msg import Joy
from morai_msgs.srv import MoraiEventCmdSrv

###################################GPS
from pyproj import Proj
from std_msgs.msg import Float32MultiArray
from morai_msgs.msg import GPSMessage
###################################GPS

class joy_tel():
    def __init__(self):
        self.msg_a = CtrlCmd()

        ############################################################################################GPS 파일을 만들기 위한 작업
        self.cnt = 0 #행렬 넣을 때 처음 값은 반드시 임시 값에 넣어주기 위한 변수
        self.x_arr = np.array([])# 1열의 무한 행 행렬
        self.y_arr = np.array([])

        #self.temp_x_arr = np.array(1)
        #self.temp_y_arr = np.array(1)
        ############################################################################################
        
        self.pub = rospy.Publisher(name="/ctrl_cmd",data_class=CtrlCmd,queue_size=1)
        self.sub = rospy.Subscriber(name="/joy",data_class=Joy,callback=self.callbackFunction)
        ###########################################################################################GPS
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.gps_callback)
        self.proj_UTM = Proj(proj='utm', zone=52, ellps = 'WGS84', preserve_units=False)

        self.utm_msg = Float32MultiArray()
        self.is_gps_data = False
        ###########################################################################################GPS
        self.srv_event_cmd = rospy.ServiceProxy('/Service_MoraiEventCmd', MoraiEventCmdSrv)

        self.event_cmd_srv = MoraiEventCmdSrv()
        self.event_cmd_srv.option = 2
        self.event_cmd_srv.ctrl_mode = 3
        self.event_cmd_srv.gear = 4

        ###################################전조등 관련
        self.lamp = Lamps()
        self.lamp.turnSignal = 1

        self.event_cmd_srv.lamps = self.lamp

        self.event_cmd_srv.emergency = 0
        self.event_cmd_srv.set_pause = False
        self.srv_event_cmd(self.event_cmd_srv) # 서비스 요청
        ###################################

    def callbackFunction(self, msg): #콜백 함수
        self.steer = msg.axes[0] * 0.7
        if (self.steer < 0.1173) and (self.steer > -0.1173): # 핸들의 민감도를 조절하여 작은 값은 무시하도록 함
             self.steer = 0 
        self.acc = msg.axes[5] # 조이스틱에 해당하는 값들 (http://wiki.ros.org/joy#Published_Topics) 참고
        self.brk = msg.axes[2]

        self.back = msg.buttons[4]
        self.go = msg.buttons[5]
        self.park = msg.buttons[0]

        ##################################### 속도, 브레이크, 핸들 제어
        self.msg_a.accel = 0.5-(self.acc/2)
        self.msg_a.steering = (self.steer)
        self.msg_a.brake = 0.5-(self.brk/2)
        #####################################

        ##################################### 기어 변경 코드(메세지 타입 안의 모든 변수의 값을 초기화 해줘야지 서비스 통신이 된다)
        if self.back == 1:
             self.event_cmd_srv.option = 2
             self.event_cmd_srv.ctrl_mode = 3
             self.event_cmd_srv.gear = 2

             self.event_cmd_srv.emergency = 0
             self.event_cmd_srv.set_pause = False
             self.srv_event_cmd(self.event_cmd_srv)

        elif self.go == 1:
             self.event_cmd_srv.option = 2
             self.event_cmd_srv.ctrl_mode = 3
             self.event_cmd_srv.gear = 4

             self.event_cmd_srv.emergency = 0
             self.event_cmd_srv.set_pause = False
             self.srv_event_cmd(self.event_cmd_srv)

        elif self.park == 1:
             self.event_cmd_srv.option = 2
             self.event_cmd_srv.ctrl_mode = 3
             self.event_cmd_srv.gear = 1

             self.event_cmd_srv.emergency = 0
             self.event_cmd_srv.set_pause = False
             self.srv_event_cmd(self.event_cmd_srv)

        #####################################

        self.pub.publish(self.msg_a)

        #rospy.loginfo(self.steer)

    def gps_callback(self, msg_g): # GPS 콜백 함수
        self.is_gps_data = True
        latitude = msg_g.latitude
        longitude = msg_g.longitude
        altitude = msg_g.altitude
        utm_xy = self.proj_UTM(longitude, latitude)
        self.utm_x = utm_xy[0]
        self.utm_y = utm_xy[1]
        map_x = self.utm_x - msg_g.eastOffset
        map_y = self.utm_y - msg_g.northOffset



        ############################# 전의 좌표와 현재 좌표의 크기가 10 cm 이상인지 확인 후 임시 변수와 행렬에 삽입(((((일단 무한한 주 행렬을 만들고 임시 1행 1열 짜리를 만든 후 거리 비교 후 그 값을 임시 행렬에 넣고 그 임시 행렬을 주 행렬에 추가)))))
        if self.cnt == 0:        
            self.temp_x = self.utm_x
            self.temp_y = self.utm_y
            self.x_arr = np.append(self.x_arr, self.temp_x)#행렬에 저장
            self.y_arr = np.append(self.y_arr, self.temp_y)#행렬에 저장

        else:
            self.dis = math.isqrt((math.trunc(self.temp_x*100) - math.trunc(self.utm_x*100))**2 + (math.trunc(self.temp_y*100) - math.trunc(self.utm_y*100))**2)
            print("dis = %f" % self.dis)

            if self.dis > 10:# 10 cm
                self.temp_x = self.utm_x
                self.temp_y = self.utm_y
                self.x_arr = np.append(self.x_arr, self.temp_x)#행렬에 저장
                self.y_arr = np.append(self.y_arr, self.temp_y)#행렬에 저장
                print(self.x_arr)
                
        
        #############################

        
        self.cnt = self.cnt + 1 ########처음 제외하고는 거리를 비교하게 만들어 줌
    ################################################거리 계산해서 10 cm 이하일 때만 행렬에 저장하는 함수
    #def gps_cal(self):
    #    a = 0
    ################################################


def main():
    a = joy_tel()
    rospy.init_node("joy_sub_pub")
    r = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        r.sleep()
    
    if rospy.is_shutdown():#종료하면 저장
        path=pd.DataFrame({'x':a.x_arr,'y':a.y_arr})

        path.to_csv('path.csv', header=False, index=False)
        print("gooooood")
    
if __name__ == '__main__':
        main()       