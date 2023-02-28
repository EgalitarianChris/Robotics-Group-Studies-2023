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






NOTE: WHEN WRITING THIS PROGRAM PLEASE RE-UPLOAD TO THE GITHUB AS Controller_V1.py and increment as needed


"""







#from stable_baselines3 import PPO


import socket, pickle, Encoders












# --------------------------------------------------------------------------------- main loop


def main():


 TCP_connection, TCP_socket = connect()


 # model = PPO.load("test_PPO_model_data") - load the trained Machine Learning Agent here







 while True:


   angle_info = get_angle_info()


   Nao_info = get_Nao_info(TCP_connection, TCP_socket)


   # action, _states = model.predict(angle_info) - Machine learning agent makes the action decisions here


   action = get_action_DUMMY() # - dummy function to send example actions for testing purposes - should be replaced in final build


   send(action)


 TCP_socket.close()


  


 Encoders.close()







  


# ----------------------------------------------------------------------------------- comms







def connect(HOST = '192.168.1.4', PORT = 10000):


   # The server's IP address: 192.168.1.100   For testing on local: 147.188.37.23


    # The port used by the server: 10000







   s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


 


   s.connect((HOST, PORT))


   print('has connected')


   conn, addr = s.accept()


   return conn, s







# fetch the angles from the decoders connected to the PC and return them in the [big,small] format


# the information from the four small encoders should be converted into a single angle so that the format is consistent with the simulation


def get_angle_info():


   angle_info = Encoders.sample() # might have to change encoders.py to change iteration rate
   large_encoder_angle = angle_info[0]
   top = 0.5*(angle_info[1]+angle_info[4])
   bottom = 0.5*(angle_info[2] + angle_info[3])
   small_encoder_angle = top + bottom
   
 return [large_encoder_angle,small_encoder_angle]












# retrieve the NAO information in the [hip_angle,knee_angle] format for the ML agent to see


def get_NAO_info(TCP_connection, TCP_socket):


   from_server = pickle.loads(TCP_connection.recv(1024))


   commands = from_server


   [hip_angle,knee_angle] = from_server


      


 return [hip_angle,knee_angle]












# send the action the agent has made to the Control_API.py instance communicating with NAO


def send(TCP_connection, action)


   TCP_connection.recv(1024)


   TCP_connection.sendall(action)# Sends back the actions


 pass












# A dummy function for testing the control system - for example return values from a list of different angle values.


def get_action_DUMMY():


 return [1,2,3,4]

