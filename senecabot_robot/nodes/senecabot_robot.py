#!/usr/bin/env python

import socket
import time
import numpy as np

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


#GEOMETRY ROBOT
R = 0.078/2.0
lx = 0.021/0.854
ly = 0.095/0.854

UDP_IP = rospy.get_param("/robot_ip")
# UDP_IP = "192.168.20.122"
OUTPUT_PORT = 8888
INPUT_PORT = 8887

MAX_TIMEOUT_COUNTER = 20

cumX = 0.0
cumY = 0.0
cumW = 0.0

cmd_vel_X = 0.0
cmd_vel_Y = 0.0
cmd_vel_W = 0.0

timeout_counter = 0

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

sock.bind(('', INPUT_PORT))
print('Conexion con el robot establecida')

def inverseKinematics(Vx, Vy, w):    
    WFL = (1/R)*(Vx - Vy - (lx+ly)*w)
    WFR = (1/R)*(Vx + Vy + (lx+ly)*w)
    WRL = (1/R)*(Vx + Vy - (lx+ly)*w)
    WRR = (1/R)*(Vx - Vy + (lx+ly)*w)
    return  WFL, WFR, WRL, WRR

def getCommandVel(WFL, WFR, WRL, WRR):
    vels = (1/(2*np.pi))*np.array([WFL, WFR, WRL, WRR])
    vels = np.clip(vels,-1.0, 1.0)
    msg = 'S'
    for v in vels:
        if(v>0):
            msg = msg + '+' + '{0:.2f}'.format(v)
        else:
            msg = msg + '-' + '{0:.2f}'.format(abs(v))
    return msg

def send_vel2robot(Vx, Vy, w):
    WFL, WFR, WRL, WRR = inverseKinematics(Vx, Vy, w)
    cmdVel = getCommandVel(WFL, WFR, WRL, WRR)
    sock.sendto(cmdVel, (UDP_IP, OUTPUT_PORT))
    in_strs = sock.recvfrom(1024)[0][1:].split(';')
    if len(in_strs)!=5:
        return None
    try:
        delta_t = float(in_strs[0])
        ws = np.array([float(in_strs[i]) for i in range(1,5)]) * (2*np.pi)
        return delta_t/1e6, ws
    except ValueError:
        return None

def directKinematics(ws):
    Vx = (ws[0]+ws[1]+ws[2]+ws[3])*(R/4.0)
    Vy = (-ws[0]+ws[1]+ws[2]-ws[3])*(R/4.0)
    w  = (-ws[0]+ws[1]-ws[2]+ws[3])*(R/(4.0*(lx+ly)))
    return Vx, Vy, w

def callback_cmd_vel(data):
    global cmd_vel_X, cmd_vel_Y, cmd_vel_W, timeout_counter
    timeout_counter = 0
    cmd_vel_X = data.linear.x
    cmd_vel_Y = data.linear.y
    cmd_vel_W = data.angular.z
    
rospy.init_node('senecabot_robot', anonymous=True)
rospy.Subscriber("cmd_vel", Twist, callback_cmd_vel)
odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

while not rospy.is_shutdown():
    current_time = rospy.Time.now()
    timeout_counter += 1
    if timeout_counter>=MAX_TIMEOUT_COUNTER:
        cmd_vel_X = 0.0
        cmd_vel_Y = 0.0
        cmd_vel_W = 0.0
    response = send_vel2robot(cmd_vel_X, cmd_vel_Y, cmd_vel_W)
    if response!=None:
        delta_t, ws = response
        Vx, Vy, w = directKinematics(ws)
        cumX += Vx*delta_t
        cumY += Vy*delta_t
        cumW += w*delta_t
        #/odom topic
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, cumW)
        #/tf topic
        odom_broadcaster.sendTransform(
            (cumX, cumY, 0.),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )
        #pub /odom
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        # set the position
        odom.pose.pose = Pose(Point(cumX, cumY, 0.), Quaternion(*odom_quat))
        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(Vx, Vy, 0), Vector3(0, 0, w))
        odom_pub.publish(odom)

send_vel2robot(0, 0, 0)
