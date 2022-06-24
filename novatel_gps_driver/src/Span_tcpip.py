#!/usr/bin/env python
#-*- coding:utf-8 -*-	# 한글 주석을 달기 위해 사용한다.
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu, MagneticField, NavSatFix, PointCloud2
import socket
import time
from multiprocessing import Value, Process, Manager
import struct 
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from span_tcpip.msg import NovatelPosition,NovatelUtmPosition,NovatelVelocity,NovatelXYZ,NovatelCorrectedImuData,DiagnosticArray,NovatelDualAntennaHeading,NovatelHeadin2,Inspva,Inspvax,Insstdev,Psrdop2,Time
import numpy as np

# def chatterCallback(data):

    
def GPSINS():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("vectornav2/IMU", Imu, chatterCallback)

    rospy.spin()

   

def tcpip(current_lat, current_lon, current_alt, current_accel_x,
    current_accel_y, current_accel_z,current_vel_x, current_vel_y,current_vel_z,
    current_quat_x, current_quat_y, current_quat_z,
    current_quat_w,current_yaw,
    obj1_dist,obj2_dist,obj3_dist,obj4_dist,
    obj1_x_cent,obj2_x_cent,obj3_x_cent,obj4_x_cent,
    obj1_y_cent,obj2_y_cent,obj3_y_cent ,obj4_y_cent,
    obj1_x_min,obj2_x_min,obj3_x_min,obj4_x_min,
    obj1_x_max,obj2_x_max,obj3_x_max,obj4_x_max,
    obj1_y_min,obj2_y_min,obj3_y_min,obj4_y_min,
    obj1_y_max,obj2_y_max,obj3_y_max,obj4_y_max,
    IMU_CTC, IMU_CNT, GPS_CTC,GPS_CNT,LIDAR_CTC,LIDAR_CNT,
    LIDAR_obj_1,LIDAR_obj_2,LIDAR_obj_3,LIDAR_obj_4,kalman_yaw):
    
    #host = "192.168.1.99" # 서버 컴퓨터의 ip(여기선 내 컴퓨터를 서버 컴퓨터로 사용) 
                       # 본인의 ip주소 넣어도 됨(확인방법: cmd -> ipconfig)
    port = 4567  # 서버 포트번호(다른 프로그램이 사용하지 않는 포트번호로 지정해야 함)
    host = "192.168.10.22"
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((host, port))

    # 클라이언트 접속 준비 완료
    server_socket.listen(1)

    print('echo server start')

    #  클라이언트 접속 기다리며 대기 
    client_soc, addr = server_socket.accept()

    print('connected client addr:', addr)
    print("통신 진행 중")
    # 클라이언트가 보낸 패킷 계속 받아 에코메세지 돌려줌
    while True:

        msg = struct.pack('>52d', 
        current_accel_x.value, current_accel_y.value, current_accel_z.value, 
        current_vel_x.value, current_vel_y.value,current_vel_z.value, 
        current_quat_x.value, current_quat_y.value,current_quat_z.value, 
        current_quat_w.value, 
        current_lat.value, current_lon.value, current_alt.value, kalman_yaw.value,
        obj1_dist.value, obj1_x_cent.value, obj1_y_cent.value, obj1_x_min.value,
        obj1_x_max.value, obj1_y_min.value,obj1_y_max.value,
        obj2_dist.value, obj2_x_cent.value, obj2_y_cent.value, obj2_x_min.value,
        obj2_x_max.value, obj2_y_min.value,obj2_y_max.value,
        obj3_dist.value, obj3_x_cent.value, obj3_y_cent.value, obj3_x_min.value,
        obj3_x_max.value, obj3_y_min.value,obj3_y_max.value,
        obj4_dist.value, obj4_x_cent.value, obj4_y_cent.value, obj4_x_min.value,
        obj4_x_max.value, obj4_y_min.value,obj4_y_max.value,
        IMU_CTC.value, IMU_CNT.value, GPS_CTC.value,GPS_CNT.value,
        LIDAR_CTC.value,LIDAR_CNT.value,
        LIDAR_obj_1.value,LIDAR_obj_2.value,
        LIDAR_obj_3.value,LIDAR_obj_4.value
        )
        
        client_soc.sendall(msg)
        data = client_soc.recv(65535)

    print("통신 끝")
    server_socket.close() # 사용했던 서버 소켓을 닫아줌"""


if __name__ == '__main__':
    current_lat = Value('d', 0.0)
    current_lon = Value('d', 0.0)
    current_alt = Value('d', 0.0)
    current_accel_x = Value('d', 0.0)
    current_accel_y = Value('d', 0.0)
    current_accel_z = Value('d', 0.0)
    current_vel_x = Value('d', 0.0)
    current_vel_y = Value('d', 0.0)
    current_vel_z = Value('d', 0.0)
    current_quat_x = Value('d', 0.0)
    current_quat_y = Value('d', 0.0)
    current_quat_z = Value('d', 0.0)
    current_quat_w = Value('d', 0.0)
    current_yaw = Value('d', 0.0)
    

    #obj = [dist,x_cent, y_cent, x_min,x_max, y_min, y_max]
    obj1_dist = Value('d', 0.0)
    obj2_dist = Value('d', 0.0)
    obj3_dist = Value('d', 0.0)
    obj4_dist = Value('d', 0.0)
    
    obj1_x_cent = Value('d', 0.0)
    obj2_x_cent = Value('d', 0.0)
    obj3_x_cent = Value('d', 0.0)
    obj4_x_cent = Value('d', 0.0)

    obj1_y_cent = Value('d', 0.0)
    obj2_y_cent = Value('d', 0.0)
    obj3_y_cent = Value('d', 0.0)
    obj4_y_cent = Value('d', 0.0)

    obj1_x_min = Value('d', 0.0)
    obj2_x_min = Value('d', 0.0)
    obj3_x_min = Value('d', 0.0)
    obj4_x_min = Value('d', 0.0)

    obj1_x_max = Value('d', 0.0)
    obj2_x_max = Value('d', 0.0)
    obj3_x_max = Value('d', 0.0)
    obj4_x_max = Value('d', 0.0)

    obj1_y_min = Value('d', 0.0)
    obj2_y_min = Value('d', 0.0)
    obj3_y_min = Value('d', 0.0)
    obj4_y_min = Value('d', 0.0)

    obj1_y_max = Value('d', 0.0)
    obj2_y_max = Value('d', 0.0)
    obj3_y_max = Value('d', 0.0)
    obj4_y_max = Value('d', 0.0)

    IMU_CTC    = Value('d', 0.0)
    IMU_CNT    = Value('d', 0.0)
    GPS_CTC    = Value('d', 0.0)
    GPS_CNT    = Value('d', 0.0)
    LIDAR_CTC  = Value('d', 0.0)
    LIDAR_CNT  = Value('d', 0.0)
    
    LIDAR_obj_1 = Value('d', 0.0)
    LIDAR_obj_2 = Value('d', 0.0)
    LIDAR_obj_3 = Value('d', 0.0)
    LIDAR_obj_4 = Value('d', 0.0)

    current_roll = Value('d', 0.0)
    current_pitch = Value('d', 0.0)

    kalman_yaw = Value('d', 0.0)




    

    th1 = Process(target=GPSINS, args=())


    # th2 = Process(target=tcpip, args=(current_lat, current_lon, current_alt, current_accel_x,
    # current_accel_y, current_accel_z,current_vel_x, current_vel_y,current_vel_z,
    # current_quat_x, current_quat_y, current_quat_z,
    # current_quat_w,current_yaw,obj1_dist,obj2_dist,obj3_dist,obj4_dist,
    # obj1_x_cent,obj2_x_cent,obj3_x_cent,obj4_x_cent,
    # obj1_y_cent,obj2_y_cent,obj3_y_cent ,obj4_y_cent,
    # obj1_x_min,obj2_x_min,obj3_x_min,obj4_x_min,
    # obj1_x_max,obj2_x_max,obj3_x_max,obj4_x_max,
    # obj1_y_min,obj2_y_min,obj3_y_min,obj4_y_min,
    # obj1_y_max,obj2_y_max,obj3_y_max,obj4_y_max,
    # IMU_CTC, IMU_CNT, GPS_CTC,GPS_CNT,LIDAR_CTC,LIDAR_CNT,
    # LIDAR_obj_1,LIDAR_obj_2,LIDAR_obj_3,LIDAR_obj_4,kalman_yaw
    # ))


    # th3 = Process(target=vn100_Kalman_filter, args = (current_lat, current_lon, current_alt, current_accel_x,
    # current_accel_y, current_accel_z,current_vel_x, current_vel_y,current_vel_z,current_yaw,current_roll,current_pitch,GPS_CNT,kalman_yaw,GPS_CTC))

    th1.start()
    # th2.start()
    # th3.start()



    
    
