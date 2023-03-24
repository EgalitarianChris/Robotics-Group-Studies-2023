#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 28 17:04:36 2023

@author: demo
"""

"""

This program should:


- be written with python 3.5+ syntax


- recieve the NAO info about its own joint angles


- give this information to the trained ML agent and save its decision / action


- send the action to the computer communicating with NAO

"""
# from stable_baselines3 import PPO
import socket, pickle, Encoders, numpy as np, time, csv


# --------------------------------------------------------------------------------- main loop


def main():
    
      log = []
      decision_rate = 10
      start_timer_duration = 20
      observation = [23,25,28,37]
      
      print("trying to connect to ML server")
      internal_connection = internal_connect()
      print("conection to ML Server established")
      
      print("trying to connect to Nao server")
      TCP_connection = connect() #connect to the laptop and store the connection variable as TCP_connection
      print("connection to Nao server established")
      start_time = time.time()
      previous_timestamp = time.time()
      
      while time.time() - start_time < start_timer_duration:
          angle_info = get_angle_info(observation[2:4],decision_rate)
          previous_timestamp = clock_tick(previous_timestamp,decision_rate)
          print("countdown: ",round(start_timer_duration - (time.time() - start_time),2),"angle:",angle_info[0])
      
      start_time = time.time()
      previous_timestamp = time.time()
      
      while True: #keeps repeating so that a live feed is created
          
         angle_info = get_angle_info(observation[2:4],decision_rate) #gets the encoder data from the usb
         try:
             Nao_info = get_NAO_info(TCP_connection) #receives nao data from the laptop
         except:
             print("Nao connection lost")
             break
         
         observation = np.array([Nao_info[0],Nao_info[1],angle_info[0], angle_info[1], Nao_info[2], Nao_info[3], angle_info[2], angle_info[3]])
         timestamp = time.time()-start_time
#         print(angle_info[4:],"(angles)",timestamp)
         print(observation[3],observation[7])
         
         try:
             action = get_action_from_ML(observation,internal_connection)
             send(TCP_connection, [float(a) for a in action]) # sends the actions for the robot to do back to the laptop
         except:
             print("ML connection lost")
             break
         
         log.append([timestamp]+list(observation)+[float(a) for a in action]+[angle_info[a] for a in range(4,8)])
         previous_timestamp = clock_tick(previous_timestamp,decision_rate)
    
      Log(log,"Logs/EP test - 10 degrees - no bar - log.csv")
      TCP_connection.close()
      Encoders.close()



def clock_tick(previous_timestamp,decision_rate):
    while time.time() < previous_timestamp + 1 / decision_rate:
        pass
    return time.time()

def Log(log,filename="Logs/log.csv"):
    print("saving data")
    with open(filename,"w") as csv_file:
        csv_writer = csv.writer(csv_file)
        for row in log:
            csv_writer.writerow(row)
    print("saved data to:",filename)


# ----------------------------------------------------------------------------------- comms

def connect(HOST = '192.168.1.7', PORT = 10000): #use 192.168.1.7

   s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) #stores the socket as s
   s.connect((HOST, PORT)) #connects to the server using the socket

   print('connected')

   return s #returns the socket variable so that it can be used later

def internal_connect(HOST = '127.0.0.1', PORT = 9999): #use 192.168.1.7

   s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) #stores the socket as s
   s.connect((HOST, PORT)) #connects to the server using the socket

   print('connected internal')

   return s

def get_angle_info(previous,decision_rate): 
   

   angle_info_1 = Encoders.sample() # might have to change encoders.py to change iteration rate
#   time.sleep(0.01)
#   angle_info_2 = Encoders.sample()
   
#   large_encoder_angle_1 = angle_info_1[0]
   #top_1 = 0.5*(angle_info_1[1]+angle_info_1[4])
  # bottom_1 = 0.5*(angle_info_1[2] + angle_info_1[3])
  # small_encoder_angle_1 = top_1 + bottom_1
#   time_1 = angle_info_1[5]
   
#   large_encoder_angle_2 = angle_info_2[0]
   #top_2 = 0.5*(angle_info_2[1]+angle_info_2[4])
   #bottom_2 = 0.5*(angle_info_2[2] + angle_info_2[3])
   #small_encoder_angle_2 = top_2 + bottom_2
#   time_2 = angle_info_2[5]
   
   large_encoder_speed = (angle_info_1[0] - previous[0]) * decision_rate
   
   if previous[0] == 28 and previous[1] == 37:
       small_encoder_speed = 0
   else:
       small_encoder_speed = (angle_info_1[1] - previous[1]) * decision_rate
   
   return [angle_info_1[0],angle_info_1[1], large_encoder_speed, small_encoder_speed] + [angle_info_1[a] for a in range(1,5)] #returns the large encoder angle and the average of the small encoder angles


def get_NAO_info(TCP_connection):
    
   from_server = pickle.loads(TCP_connection.recv(1024)) #receives data from server which should be in the form of [hip_angle, knee_angle]
   nao_data = from_server
   TCP_connection.sendall(pickle.dumps('received', protocol = 0))
        
   return nao_data

def get_action_from_ML(observation,internal_connection):
    
   internal_connection.sendall(pickle.dumps(observation, protocol = 0))
   from_server = pickle.loads(internal_connection.recv(1024)) #receives data from server which should be in the form of [hip_angle, knee_angle]
   
   return from_server


def send(TCP_connection, action):

   from_server = pickle.loads(TCP_connection.recv(1024)) 
   payload = pickle.dumps(action, protocol = 0)
   TCP_connection.sendall(payload) # Sends back the actions, protocol = 0 is needed because the laptop is running python 2.7

main()
