# -*- coding: utf-8 -*-
"""
Created on Thu Feb 23 14:54:55 2023

@author: princ
"""

'''Server which gets the angles of all of the encoders and sends them to a client'''
############# CAN BE USED IN CONJUNCTION WITH  #############
############# REQUIRES Encoders.py TO BE IN THE SAME DIRECTORY AS THIS PROGRAM #############
 
import socket, pickle
 
#socket, pickle, Encoders,
 
HOST = ''#192.168.1.100'  # The server's IP address: 192.168.1.100   For testing on local: 147.188.37.23
PORT = 10000            # The port used by the server: 10000
 
print ('Server IP: ', HOST)
 
def connect():
    print ('Waiting for client connection...')
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
   
    s.bind((HOST, PORT))
   
    s.listen()
    print('before')
    conn, addr = s.accept()
    return conn, s
 
 
def get_encoder_data(conn):
   
    conn.sendall(pickle.dumps('Please send data'))    
    data = pickle.loads(conn.recv(4096))
    
    return data
 
 
def close(conn, s):
    conn.close()
    s.close()
    
conn, s = connect()
while True:
    data = get_encoder_data(conn)
    print (data)
    if data == 'close':
        break
close(conn, s)
