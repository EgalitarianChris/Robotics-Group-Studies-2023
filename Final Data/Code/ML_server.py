import socket, pickle
import numpy as np
from stable_baselines3 import PPO
import csv

def main(mode = "ML",use_small_encoders=True):
    print("mode: ",mode)
    count = 0
    
    if mode == "ML":
        model = PPO.load("/home/demo/Desktop/PythonCOde/Test Agents/Rule_based_NSC",
                         print_system_info=True,
                         custom_objects={'lr_schedule': 0.9, 'clip_range': 0})
    elif mode == "EP":
        previous_action = np.array([0,0,0,0])
    else:
        AS = load_AS("AS.csv")
        length = len(AS)
        
    connection = connect()
    print("connection established")
    
    while True:
        try:
            observation = get_obs(connection,use_small_encoders)
            
            if mode == "ML":
                action, _states = model.predict(observation)
            elif mode == "EP":
                action, previous_action = programatic(observation, previous_action)
            else:
                action = AS[count%length]
            
            send(connection,action)
            print(action,"(action) sent")
        except Exception as e:
            raise(e)
            print('connection to controller lost')
            print('trying to reconnect')
            connection = connect()
            print("connection established")
        count += 1

def load_AS(filename):
    actions = []
    with open(filename,newline="") as csv_file:
        reader = csv.reader(csv_file)
        for action in reader:
            actions.append(action)
    return actions
                   

def programatic(observation, old = np.array([0,0,0,0])):
   action = old
   if 10 > np.abs(observation[6]) and observation[2] < 0:
       action = np.array([-1,0.6,0.8,65/126])   
   elif 10 > np.abs(observation[6]) and observation[2] > 0:
       action = np.array([1,0.6,-0.8,65/126])
   old_for_next = action
   return action, old_for_next

def connect():
    print ('Waiting for Controller connection...')
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("", 9999)) 
    s.listen()
    conn, addr = s.accept()
    return conn

def get_obs(TCP_connection,use_small_encoders):
   from_server = pickle.loads(TCP_connection.recv(1024)) #receives data from server which should be in the form of [hip_angle, knee_angle]    
   if not use_small_encoders:
       from_server = np.delete(from_server,[3,6])
   print(from_server)
   return from_server

def send(TCP_connection, action):
   payload = pickle.dumps(action, protocol = 0)
   TCP_connection.sendall(payload) 
   
main("EP",True)
   
   