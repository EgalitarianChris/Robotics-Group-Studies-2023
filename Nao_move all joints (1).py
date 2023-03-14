
# -*- coding: utf-8 -*-
"""
Created on Fri Mar 10 15:34:18 2023

@author: Yuusf Choudhury
"""

import time
import socket, pickle
import sys
from naoqi import ALProxy
#from torso_mover import *

def calculate_angles(A, B):  #A, B are the desired torso/leg angle respectively
    
    if A < Torso_min or A > Torso_max:
        return "Error: Torso angle out of range"
    
    if B < Leg_min or B > Leg_max:
        return "Error: Leg angle out of range"

    C = (B - Leg_min)/(Leg_max - Leg_min)

    Knee_pitch = 1.4 - 1.4905*C
    Ankle_pitch = 0.7608 - 0.8789*C

    D = (Torso_max - A) / (Torso_max - Torso_min)

    Shoulder_pitch = 1.11 + 0.34*D
    LShoulder_roll = 0.095 + 0.455*D
    RShoulder_roll = (-1)*LShoulder_roll
    LElbow_roll = 0.308 - 1.808*D
    RElbow_roll = (-1)*LElbow_roll
    LWrist_yaw = -0.713 + 0.143*D
    RWrist_yaw = (-1)*LWrist_yaw
    Hip_pitch = -0.092 - 0.645*D

    return Knee_pitch, Ankle_pitch, Shoulder_pitch, LShoulder_roll, RShoulder_roll, LElbow_roll, RElbow_roll, LWrist_yaw, RWrist_yaw, Hip_pitch

class NAOController:
    def __init__(self, robot_ip, robot_port=9559):
        # Connect to the NAO robot
        self.motion_proxy = ALProxy("ALMotion", robot_ip, robot_port)
        
        # Set joint stiffness (optional)
        self.stiffness = 1.0
        self.motion_proxy.setStiffnesses(["Body"], self.stiffness)
        
    def move_joint(self, joint_name, angle, speed):
        #angle_rad = angle * 3.14 / 180.0    #converts angle to radians if needed
        
        # Set joint angle and speed
        self.motion_proxy.setAngles(joint_name, angle, speed)
        
    def move_knee_pitch(self, angle, speed):
        self.move_joint("LKneePitch", angle, speed)
        self.move_joint("RKneePitch", angle, speed)
        
    def move_ankle_pitch(self, angle, speed):
        self.move_joint("LAnklePitch", angle, speed)
        self.move_joint("RAnklePitch", angle, speed)
        
    def move_shoulder_pitch(self, angle, speed):
        self.move_joint("LShoulderPitch", angle, speed)
        self.move_joint("RShoulderPitch", angle, speed)
        
    def move_shoulder_roll(self, angle, speed):
        self.move_joint("LShoulderRoll", angle, speed)
        self.move_joint("RShoulderRoll", -angle, speed)
        
    def move_elbow_roll(self, angle, speed):
        self.move_joint("LElbowRoll", angle, speed)
        self.move_joint("RElbowRoll", -angle, speed)
        
    def move_wrist_yaw(self, angle, speed):
        self.move_joint("LWristYaw", angle, speed)
        self.move_joint("RWristYaw", -angle, speed)
        
    def move_hip_pitch(self, angle, speed):
        self.move_joint("LHipPitch", angle, speed)
        self.move_joint("RHipPitch", angle, speed)

    
    def move_elbow_yaw(self, angle, speed):
        self.move_joint("LElbowYaw", angle, speed)
        self.move_joint("RElbowYaw", -angle, speed)
    
    def move_head_yaw(self, angle, speed):
        self.move_joint("HeadYaw", angle, speed)
        
    def move_head_pitch(self, angle, speed):
        self.move_joint("HeadPitch", angle, speed)
        
    def move_hip_yaw_pitch(self, angle, speed):
        self.move_joint("LHipYawPitch", angle, speed)
        self.move_joint("RHipYawPitch", angle, speed)
    
    def move_ankle_roll(self, angle, speed):
        self.move_joint("LAnkleRoll", angle, speed)
        self.move_joint("RAnkleRoll", angle, speed)

    def move_hip_roll(self, angle, speed):
        self.move_joint("LHipRoll", angle, speed)
        self.move_joint("RHipRoll", angle, speed)   
    

NAO_IP = "192.168.1.3"
# main loop - should'nt need to be changed much
def main(PORT_NUMBER = 10000, NAO_IP = "192.168.1.3"): # NAO IP 192.168.1.3
  
  TCP_connection, TCP_socket = TCP_connect(PORT_NUMBER)
  NAO_connection = NAO_connect(NAO_IP)


  while True:
      
    NAO_info = get_NAO_info(NAO_connection)
    # print "nao info recieved:"+str(NAO_info)
    send(NAO_info, TCP_socket, TCP_connection)
    # print "info sent"
    action = get_action(TCP_connection, TCP_socket)
    print "action recieved"+str(action)
    move_NAO(NAO_connection,action)
    # print "nao moved"
    time.sleep(0.1)

    
# -------------------------------------------------------- NAO stuff:  

# connect to NAO using its IP and port number and return the connection variable
def NAO_connect(NAO_IP = "192.168.1.3"):
    try:
        print "trying to connect"
        motionProxy = ALProxy("ALMotion", NAO_IP, 9559)
        print "connected"
    except Exception,e:
        print "Could not create proxy to ALMotion"
        print "Error was: ",e
        sys.exit(1)
    return motionProxy


# elbow yaw, head yaw, head pitch, hip yaw pitch, ankle roll, hip roll
fraction_speed = 0.7
speed = fraction_speed / 28.1   #max value is 1/28.1, can take values between -1 and 1

#define set of angles corresponding to extended/contracted position
angles1 = calculate_angles(Torso_min, Leg_min)
angles2 = calculate_angles(Torso_max, Leg_max)
angles3 = calculate_angles(10, 180)  #possible neutral position angle set

def set_neutral_position():   #sets initial neutral position
    nao.move_knee_pitch(angles3[0], 0.5)
    nao.move_ankle_pitch(angles3[1, 0.5])
    nao.move_shoulder_pitch(angles3[2], 0.5)
    nao.move_shoulder_roll(angles3[3], 0.5)
    nao.move_elbow_roll(angles3[5], 0.5)
    nao.move_wrist_yaw(angles3[7], 0.5)
    nao.move_hip_pitch(angles3[9], 0.5)
    
    nao.move_elbow_yaw(-0.9971, 0.5)
    nao.move_head_yaw(0.0168, 0.5)
    nao.move_head_pitch(0.5148, 0.5)
    nao.move_hip_yaw_pitch(-0.0367, 0.5)
    nao.move_ankle_roll(-0.0199, 0.5)
    nao.move_hip_roll(-0.0014, 0.5)


# take the action in the [hip_angle,hip_angle_speed,knee_angle,knee_angle_speed] format and turn this into movement on NAO
def move_NAO(NAO_connection,action):
    #angles are taken as fractional but nao needs absoulte angles between -0.09 and 2 rads, fractional angles are -1 to 1
    action = action_convert(action)
    print "converted action:"+str(action)
    NAO_connection.setStiffnesses("Body", 1.0)
    knee_names = ['RKneePitch', 'LKneePitch']
    hip_names  = ['RHipPitch', 'LHipPitch']
    
    if action[1] != 0:

        nao.move_knee_pitch(angles1[0], 26.8*speed)   #normalised each joint's speed
        
        nao.move_ankle_pitch(angles1[1], 15.7*speed)
        nao.move_shoulder_pitch(angles1[2], 4.77*speed)
        nao.move_shoulder_roll(angles1[3], 7.36*speed)  # -ve signs mean joint angle is decreasing
        nao.move_elbow_roll(angles1[5], 28.1*speed)
        nao.move_wrist_yaw(angles1[7], speed)
        nao.move_hip_pitch(angles1[9], 11.6*speed)
    if action[3] != 0:
        nao.move_knee_pitch(angles2[0], 26.8*speed)
        nao.move_ankle_pitch(angles2[1], 15.7*speed)
        nao.move_shoulder_pitch(angles2[2], 4.77*speed)
        nao.move_shoulder_roll(angles2[3], 7.36*speed)
        nao.move_elbow_roll(angles2[5], 28.1*speed)
        nao.move_wrist_yaw(angles2[7], speed)
        nao.move_hip_pitch(angles2[9], 11.6*speed)
    #anglesx = calculate_angles(action[2], action[0])
    #nao.move_hip_pitch(anglesx[9], 0.5*(action[3]+1))
    



# request the knee and hip angle from NAO and return it in the [hip_angle, knee_angle] format
def get_NAO_info(NAO_connection, NAO_IP = "192.168.1.3"):
    #motionProxy = ALProxy("ALMotion", NAO_IP, 9559)
    useSensors  = True
    Knee_angle = NAO_connection.getAngles('RKneePitch', useSensors)
    Hip_angle = NAO_connection.getAngles('RHipPitch', useSensors)
    return [Knee_angle[0]*180/3.14159,Hip_angle[0]*180/3.14159]

def action_convert(action, l0 = 1.185, l1 = -0.092, h0 = -0.35, h1 =-1.033):    #variables are min max leg and hip angle
    action = [action[0]*(l1 - l0)/2 + l0 + (l1 - l0)/2,
              action[1]*1/2 + 1/2,
              action[2]*(h1 - h0)/2 + h0 + (h1 - h0)/2,
              action[3]*1/2 + 1/2]
    #maximum upright hip angle [-1.0338740348815918]
    #minimum knee pitch [1.1857401132583618]
    #maximum lean back for hip angle [-0.35]
    #barnaby code in new hip angle for leaning back
    return action
# -------------------------------------------------------- Client - server stuff

# initialise the connection between the client and the server using the port number
def TCP_connect(PORT_NUMBER):
  print ('Waiting for client connection...')
  s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
   
  s.bind(("", PORT_NUMBER))
   
  s.listen(5)
  conn, addr = s.accept()
  print ('Client connected')
  return conn, s
  

# recieve the action info from the controller.py instance running on the lab PC and return this info in the:
# [hip_angle,hip_angle_speed,knee_angle,knee_angle_speed] format
def get_action(TCP_connection, TCP_socket):
  TCP_connection.sendall(pickle.dumps('Please send data',  protocol = 0)) 
  r = TCP_connection.recv(1024)
  data = pickle.loads(r)
  return data
  


# send the NAO_info array in [hip_angle, knee_angle] format to the lab PC
def send(NAO_info, s, TCP_connection):
  data = pickle.dumps(NAO_info, protocol=0) #used for testing
  TCP_connection.sendall(data)
  B = pickle.loads(TCP_connection.recv(1024)) #used for testing
  

  
main()