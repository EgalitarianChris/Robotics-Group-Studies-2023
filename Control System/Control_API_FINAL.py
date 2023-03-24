import time
import socket, pickle
import sys
from naoqi import ALProxy

NAO_IP = "192.168.1.3"

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

   
    
    
    
# -------------------------------------------------------- NAO stuff:  

#connect to NAO using its IP and port number and return the connection variable
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

# def NAO_connect(NAO_IP = "192.168.1.3"):
#     return None

# take the action in the [hip_angle,hip_angle_speed,knee_angle,knee_angle_speed] format and turn this into movement on NAO
def move_NAO(NAO_connection, action):
    #angles are taken as fractional but nao needs absoulte angles between -0.09 and 2 rads, fractional angles are -1 to 1
    action = action_convert(action)
    print "converted action" + str(action)
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

    start_time = time.time()
    useSensors  = True
    Knee_angle_initial = NAO_connection.getAngles('RKneePitch', useSensors)[0]
    Hip_angle_initial = NAO_connection.getAngles('RHipPitch', useSensors)[0]

    time.sleep(0.05)
    Knee_angle_finial = NAO_connection.getAngles('RKneePitch', useSensors)[0]
    Hip_angle_finial = NAO_connection.getAngles('RHipPitch', useSensors)[0]
    end_time = time.time()
    
    knee_speed = (Knee_angle_finial - Knee_angle_initial) / (end_time - start_time)
    hip_speed = (Hip_angle_finial - Hip_angle_initial) / (end_time - start_time)
    

    
    #print("speeds:" + str(knee_speed)+" , "+str(hip_speed))
    return [Knee_angle_initial*180/3.14159,Hip_angle_initial*180/3.14159, knee_speed*180/3.14159, hip_speed*180/3.14159]


def action_convert(action, l0 = 1.185, l1 = -0.092, h0 = -0.35, h1 =-1.033):    #variables are min max leg and hip angle
    action = [action[0]*(l1 - l0)/2 + l0 + (l1 - l0)/2,
              action[1]*0.5 + 0.5,
              action[2]*(h1 - h0)/2 + h0 + (h1 - h0)/2,
              action[3]*0.5 + 0.5]
    #maximum upright hip angle [-1.0338740348815918]
    #minimum knee pitch [1.1857401132583618]
    #maximum lean back for hip angle [-0.35]
    #barnaby (En)code in new hip angle for leaning back
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
