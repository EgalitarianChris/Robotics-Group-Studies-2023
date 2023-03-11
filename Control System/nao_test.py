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

# take the action in the [hip_angle,hip_angle_speed,knee_angle,knee_angle_speed] format and turn this into movement on NAO
def move_NAO(NAO_connection,action):
    #angles are taken as fractional but nao needs absoulte angles between -0.09 and 2 rads, fractional angles are -1 to 1
    action = action_convert(action)
    print "converted action:"+str(action)
    NAO_connection.setStiffnesses("Body", 1.0)
    knee_names = ['RKneePitch', 'LKneePitch']
    hip_names  = ['RHipPitch', 'LHipPitch']
    
    if action[1] != 0:
        NAO_connection.setAngles(knee_names, [action[0], action[0]], action[1])
    if action[3] != 0:
        NAO_connection.setAngles(hip_names, [action[2], action[2]], action[3])
    #anglesx = calculate_angles(action[2], action[0])
    #nao.move_hip_pitch(anglesx[9], 0.5*(action[3]+1))
    
    


# request the knee and hip angle from NAO and return it in the [hip_angle, knee_angle] format
def get_NAO_info(NAO_connection, NAO_IP = "192.168.1.3"):
    #motionProxy = ALProxy("ALMotion", NAO_IP, 9559)
    useSensors  = True
    Knee_angle = NAO_connection.getAngles('RKneePitch', useSensors)
    Hip_angle = NAO_connection.getAngles('RHipPitch', useSensors)
    return [Knee_angle[0]*180/3.14159,Hip_angle[0]*180/3.14159]

def action_convert(action):
    action_degrees = [action[0]*36.6128 + 31.3228,
                       action[1]*190.3 + 190.3,
                       action[2]*21.1365 - 38.0955,
                       action[3]*190.3 + 190.3]
    action = [action_degrees[0]*3.14159/180,action_degrees[1]/380.6,action_degrees[2]*3.14159/180,action_degrees[3]/380.6]
    #maximum upright hip angle [-1.0338740348815918]   -59.232
    #minimum knee pitch [1.1857401132583618]  -5.29   67.9356
    #maximum lean back for hip angle [-0.35]   -16.959
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