'''defining the and constructing the simulation - to be used by the ML file in the same github folder'''
import pymunk
import numpy as np
import pygame


class Rod:
    'class to construct the top rod of the simulation'
    def __init__(self, pos, a, b, m, space, radius=2):
        'position of CoM, start, end, mass, space, radius(width)'
        
        self.body = pymunk.Body()
        self.body.position = pos
        
        #properties of the rod
        rod = pymunk.Segment(self.body, a, b, radius)
        rod.mass = m
        rod.filter = pymunk.ShapeFilter(group=1)
        rod.color = (0, 255, 0, 0)
        
        space.add(self.body, rod)


class Leg:
    'class to construct the leg'
    def __init__(self, pos, a1, b1, a2, b2, m1, m2, space, radius=2):
        'position of CoM, leg_start, leg_end, foot_start, foot_end,'
        ' leg_mass,  foot_mass, space, radius(width)'
        
        self.body = pymunk.Body()
        self.body.position = pos
        
        #properties of the leg
        leg = pymunk.Segment(self.body, a1, b1, radius=2)
        leg.filter = pymunk.ShapeFilter(group = 1)
        leg.color = (0, 255, 0, 0)
        leg.mass = m1
        
        #properties of the foot
        foot= pymunk.Segment(self.body, a2, b2, radius=1)
        foot.filter = pymunk.ShapeFilter(group = 1)
        leg.mass = m2
        foot.color = (0, 255, 0, 0)
        
        space.add(self.body, leg, foot)


class Swing:
    'class that constructs the swing and Naos attatched leg segment'
    def __init__(self,pos, a1, b1, a2, b2, a3, b3, a4, b4, m1, m2, m3, m4, space):
        'position of CoM, a=start, b=end, m=mass, 1/2/3/4=bar/vertical/base/leg segment'
        
        self.body = pymunk.Body()
        self.body.position = pos
        
        #properties of the bar (the part nao holds onto)
        bar = pymunk.Segment(self.body, a1, b1 , radius=2) #bar
        bar.filter = pymunk.ShapeFilter(group = 1)
        bar.mass = m1
        
        #properties of the vertical segment of the swing
        vertical = pymunk.Segment(self.body, a2, b2, radius=3) #vertical
        vertical.filter = pymunk.ShapeFilter(group = 1)
        vertical.mass = m2
        
        #properties of the base of the swing
        base = pymunk.Segment(self.body, a3, b3, radius=1) #base
        base.filter = pymunk.ShapeFilter(group = 1)
        base.mass = m3
        
        #properties of nao upper leg attatched to the swing
        leg = pymunk.Segment(self.body, a4, b4, radius=3) #upper leg
        leg.filter = pymunk.ShapeFilter(group = 1)
        leg.mass = m4
        leg.color = (0, 255, 0, 0)
        
        space.add(self.body, bar, vertical, base, leg)


class Torso:
    'constructing Naos torso and head'
    def __init__(self, pos, a1, b1, r2, a2, a3, b3, m1, m2, m3, space):
        'position of CoM, a1= torso start, b1=torso end, r2=head radius, a2=head offset, m=mass'

        self.body = pymunk.Body()
        self.body.position = pos

        # properties of the torso
        self.torso = pymunk.Segment(self.body, a1, b1, radius=2)
        self.torso.filter = pymunk.ShapeFilter(group=1)
        self.torso.mass = m1
        self.torso.color = (255, 0, 0, 0)

        # properties of the head
        self.head = pymunk.Circle(self.body, r2, a2)
        self.head.mass = m2
        self.head.filter = pymunk.ShapeFilter(group=1)
        self.head.color = (200, 200, 0, 0)

        # properties of the arms
        self.arms = pymunk.Segment(self.body, a3, b3, radius=1.8)
        self.arms.mass = m3
        self.arms.filter = pymunk.ShapeFilter(group=1)
        self.arms.color = (200, 0, 200, 0)

        space.add(self.body, self.torso, self.head, self.arms)

class Simplemotor:
    'class to add a motor of constant speed to a joint (automatically replaces old motors)'
    def __init__(self, body1, body2, rate, space, switch="off"):
        'rate is angular velocity in radians'
        
        self.simplemotor = pymunk.SimpleMotor(body1, body2, rate)
        space.add(self.simplemotor)


class Pivotjoint:
    def __init__(self, body1, body2, con1, con2, space, err = pow(1-0.1,30)):
        'two bodies and where to connect them relative to the centre of each body, changing error bias allows speed change for simulation'
        
        joint = pymunk.constraints.PivotJoint(body1, body2, con1, con2)
        joint.error_bias = pow(1-0.4, 60)
        space.add(joint)


class RotaryLimitJoint:
    'adds an angle limit to a previously existing joint'
    def __init__(self, body1, body2, min, max, space):
        'prevents joint exceeding maximum angle'
        
        joint = pymunk.constraints.RotaryLimitJoint(body1, body2, min, max)
        space.add(joint)

        
class Dampedrotaryspring:
    'adds damping to a previously existing joint'
    def __init__(self, body1, body2, angle, stiff, damp, space):
        'two bodies and, a rest angle, a stiffness and a damping factor.'
        
        # only really used for top joint and it wouldnt work well for others,
        joint = pymunk.constraints.DampedRotarySpring(body1, body2, angle, stiff, damp)
        space.add(joint)

#----------------------------------------------------------------------------------------------------------------------
# FUNCTIONS FOR USE IN THE GYM ENVIRONMENT (PREVIOUSLY WITHIN GYM WITH SIM):

def setup_simulation():
    '''Adds bodies and joints to the simulation space and returns a dictionary of system information'''
    
    #defining the space and some useful variables
    pm_space = pymunk.Space()
    pm_space.gravity = 0, 981
    background = pm_space.static_body
    speeds = [0, 0]
    step_length = 1/ 100
    
    setup = {
        #other variables
        "bg": (400, 200), #position of the top joint
        "step_length": step_length, #time step of the simulation
        "sim_steps_per_decision": 10, #number of simulation steps per action taken by machine learning
        "phi": -np.pi / 12, #initial angle of the top joint
        
        # lengths /cm
        "rl": 151 + 7, #rod length
        "sl1": 16, #swing bar length
        "sl2": 19 + 5, #swing vertical length
        "sl3": 17, #swing base length
        "sl4": [8,10], #nao upper leg [width, length]
        "ll1": 14, #lower leg length
        "ll2": 15, #foot lenth
        "tl": 18.5, #torso length
        "al": 26.0, #arm length

        # masses /kg (numbered same as lengths)
        "rm": 1.235 + 0.381 / 2,
        "sm1": 0.381 / 2,
        "sm2": 1.026,
        "sm3": 0.131 * 2 + 0.070 + 0.390 * 2,
        "sm4": 0.603* 2 + 0.134,
        "lm1": 0.292 * 2 + 0.134,
        "lm2": 0.162 * 2 + 0.134,
        "tm": 1.050 + 0.064 + 0.075 + 0.070,
        "head": 0.605,
        "arms": 0.07504 + 0.15794 + 0.15777 + 0.06483*2 + 0.0777*2 + 0.18533*2
    }
    
    centres = {
        #'centre' of each body defined from the top of the swing (not centre of mass, arbitrary point often the centre of the main segment of a body)
        "rc": (setup["bg"][0] + np.sin(setup["phi"])*setup["rl"] / 2, setup["bg"][1] + np.cos(setup["phi"])*setup["rl"] / 2),
        "sc": (setup["bg"][0] + np.sin(setup["phi"])*setup["rl"] + np.sin(setup["phi"])*setup["sl2"] /2, setup["bg"][1] + np.cos(setup["phi"])*setup["rl"] + np.cos(setup["phi"])*setup["sl2"] / 2),
        "lc": (setup["bg"][0] + np.sin(setup["phi"])*setup["rl"] + np.sin(setup["phi"])*(setup["sl2"] - setup["sl4"][0]) + np.cos(setup["phi"])*setup["sl4"][1] + np.sin(setup["phi"])*setup["ll1"], setup["bg"][1] + np.cos(setup["phi"])*setup["rl"] + np.cos(setup["phi"])*(setup["sl2"] - setup["sl4"][0]) - np.sin(setup["phi"])*setup["sl4"][1] + np.cos(setup["phi"])*setup["ll1"]),
        "hip": (setup["bg"][0] + np.sin(setup["phi"])*(setup["rl"] + setup["sl2"] - setup["sl4"][0] /2), setup["bg"][1] + np.cos(setup["phi"])*(setup["rl"] + setup["sl2"] - setup["sl4"][0] /2)),
        # "a1c": (setup["bg"][0] + setup["a1"]*np.cos(np.pi*(90-30.74)/180) / 2, setup["bg"][1] + setup["rl"] + setup["sl2"] - setup["tl"] + setup["a1"]*np.sin(np.pi*(90-30.74)/180) / 2),
        # "a2c": (setup["bg"][0] + setup["a1"]*np.cos(np.pi*(90-30.74)/180) + setup["a2"]*np.cos(np.pi*(54.5-30.74)/180) / 2, setup["bg"][1] + setup["rl"] + setup["sl2"] - setup["tl"] + setup["a1"]*np.sin(np.pi*(90-30.74)/180) - setup["a2"]*np.sin(np.pi*(54.5-30.74)/180) / 2)
    }

    bodies = {
        # Bodies added to the simulation (centre then start and end of each segment as defined in the classes)
        "rod": Rod(centres["rc"], (-np.sin(setup["phi"])*setup["rl"] / 2, -np.cos(setup["phi"])*setup["rl"] / 2), (np.sin(setup["phi"])*setup["rl"] / 2, np.cos(setup["phi"])*setup["rl"] / 2), setup["rm"], pm_space),
        "swing": Swing(centres["sc"],
                           (-np.sin(setup["phi"])*setup["sl2"] /2, -np.cos(setup["phi"])*setup["sl2"] / 2), (np.cos(setup["phi"])*setup["sl1"] - np.sin(setup["phi"])*setup["sl2"] /2, -np.cos(setup["phi"])*setup["sl2"] / 2 - np.sin(setup["phi"])*setup["sl1"]),#a1,b1
                           (-np.sin(setup["phi"])*setup["sl2"] /2, -np.cos(setup["phi"])*setup["sl2"] / 2), (np.sin(setup["phi"])*setup["sl2"] /2, np.cos(setup["phi"])*setup["sl2"] / 2),#a2,b2 
                           (-np.cos(setup["phi"])*(setup["sl3"] / 2 - 0.5) + np.sin(setup["phi"])*setup["sl2"] /2, np.sin(setup["phi"])*(setup["sl3"] / 2 - 0.5) + np.cos(setup["phi"])*setup["sl2"] / 2), (np.cos(setup["phi"])*(setup["sl3"] / 2 - 0.5) + np.sin(setup["phi"])*setup["sl2"] /2, -np.sin(setup["phi"])*(setup["sl3"] / 2 - 0.5) + np.cos(setup["phi"])*setup["sl2"] / 2),#a3,b3
                           (np.sin(setup["phi"])*(setup["sl2"] / 2 - setup["sl4"][0] /2) , np.cos(setup["phi"])*(setup["sl2"] / 2 - setup["sl4"][0] /2)), (np.sin(setup["phi"])*(setup["sl2"] / 2 - setup["sl4"][0] /2) + np.cos(setup["phi"])*setup["sl4"][1], np.cos(setup["phi"])*(setup["sl2"] / 2 - setup["sl4"][0] /2) - np.sin(setup["phi"])*setup["sl4"][1]),#a4,b4
                           setup["sm1"], setup["sm2"], setup["sm3"], setup["sm4"], pm_space),
        "leg": Leg(centres["lc"],
                           (-np.sin(setup["phi"])*setup["ll2"] /2, -np.cos(setup["phi"])*setup["ll2"] /2), (np.sin(setup["phi"])*setup["ll2"] /2, np.cos(setup["phi"])*setup["ll2"] /2),
                           (np.sin(setup["phi"])*setup["ll2"] /2 - np.cos(setup["phi"])*5, np.cos(setup["phi"])*setup["ll2"] /2 + np.sin(setup["phi"])*5), (np.sin(setup["phi"])*setup["ll2"] /2 + np.cos(setup["phi"])*10, np.cos(setup["phi"])*setup["ll2"] /2 - np.sin(setup["phi"])*10),
                           setup["lm1"], setup["lm2"], pm_space),
        "torso": Torso((centres["hip"][0] - np.sin(setup["phi"]+np.pi/4) * (setup["tl"] / 2 - 0.25),
                        centres["hip"][1] - np.cos(setup["phi"]+np.pi/4) * (setup["tl"] / 2 - 0.25)),
                       (- np.sin(setup["phi"]+np.pi/4) * (setup["tl"] / 2 + 0.25),
                        - np.cos(setup["phi"]+np.pi/4) * (setup["tl"] / 2 + 0.25)), 
                       (np.sin(setup["phi"]+np.pi/4) * (setup["tl"] / 2 + 0.25),
                       np.cos(setup["phi"]+np.pi/4) * (setup["tl"] / 2 + 0.25)),  # a1,b1
                       6, (- np.sin(setup["phi"]+np.pi/4) * (2 + setup["tl"] / 2 + 0.25),
                           - np.cos(setup["phi"]+np.pi/4) * (setup["tl"] / 2 + 0.25)), #head
                       (- np.sin(setup["phi"]+np.pi/4) * (2 + setup["tl"] / 2 + 0.25 - 3.5),
                        - np.cos(setup["phi"]+np.pi/4) * (setup["tl"] / 2 + 0.25 - 3.5)),
                       (- np.sin(setup["phi"]+np.pi/4) * (2 + setup["tl"] / 2 + 0.25 - 3.5 + setup["al"]),
                        - np.cos(setup["phi"]+np.pi/4) * (setup["tl"] / 2 + 0.25 - 3.5 + setup["al"])),
                       setup["tm"], setup["head"], setup["arms"], pm_space),
    }

    joints = {
        # Joints between the bodies (body 1, body 2, coordinates of joint realtive to centre of body 1, coordinates relative to body 2 centre)
        "torso": Pivotjoint(bodies["swing"].body, bodies["torso"].body,
                          (np.sin(setup["phi"])*(setup["sl2"] / 2 - setup["sl4"][0] /2) , np.cos(setup["phi"])*(setup["sl2"] / 2 - setup["sl4"][0] /2)),
                          (np.sin(setup["phi"]+np.pi/4)*(setup["tl"] / 2 +0.25), np.cos(setup["phi"]+np.pi/4)*(setup["tl"] / 2 +0.25)),
                          pm_space),
        "leg": Pivotjoint(bodies["swing"].body, bodies["leg"].body,
                            (np.sin(setup["phi"])*(setup["sl2"] / 2 - setup["sl4"][0] /2) + np.cos(setup["phi"])*setup["sl4"][1], np.cos(setup["phi"])*(setup["sl2"] / 2 - setup["sl4"][0] /2) - np.sin(setup["phi"])*setup["sl4"][1]),
                            (-np.sin(setup["phi"])*setup["ll2"] /2, -np.cos(setup["phi"])*setup["ll2"] /2),
                            pm_space),
        "swing": Pivotjoint(bodies["rod"].body, bodies["swing"].body,
                            (np.sin(setup["phi"])*setup["rl"] / 2, np.cos(setup["phi"])*setup["rl"] / 2),
                            (-np.sin(setup["phi"])*setup["sl2"] /2, -np.cos(setup["phi"])*setup["sl2"] / 2),
                            pm_space),
        "top": Pivotjoint(background, bodies["rod"].body, setup["bg"],
                              (-np.sin(setup["phi"])*setup["rl"] / 2, -np.cos(setup["phi"])*setup["rl"] / 2),
                              pm_space),
        
        # limit on swing joint to make the system comparable to the real world system
        "limit": RotaryLimitJoint(bodies["rod"].body, bodies["swing"].body, -np.pi/3 -setup["phi"], np.pi/3 -setup["phi"], pm_space),
        
        # damping on the top joint (tip) and swing joint (tap)
        "tip": Dampedrotaryspring(background, bodies["rod"].body, 0, 0, 1922, pm_space),
        "tap": Dampedrotaryspring(bodies["rod"].body, bodies["swing"].body, 0, 0, -92.82, pm_space)
    }
    
    motors = {
        #motors for the hip and knee assuming upper leg is fixed to the swing
        "torso": Simplemotor(bodies["swing"].body, bodies["leg"].body, 0, pm_space),
        "leg": Simplemotor(bodies["swing"].body, bodies["torso"].body, 0, pm_space),
    }
    return {"pm_space": pm_space, "motors": motors, "bodies": bodies, "joints": joints, "speeds": speeds, "setup": setup}


def perform_action(environment, action, simulation_data):
    #redefining actions to be in degrees and degrees per second from the -1, 1 range used by machine learning
    #action = (leg angle, leg speed, torso angle, torso speed)
    action = np.array([action[0]*((90--5.271) - (90-67.895))/2 + ((90--5.271) + (90-67.895))/2,
                       action[1]*190.3 + 190.3,
                       action[2]*((90-29.186) - (90-70.054))/2 + ((90-29.186) + (90-70.054))/2,
                       action[3]*190.3 + 190.3])
    
    #get angle of leg and torso relative to the swing in degrees (defined as 0 perpendicular to the swing)
    leg_angle = - 180/np.pi * (environment.simulation_data["pm_space"].bodies[2].angle - environment.simulation_data["pm_space"].bodies[1].angle)
    torso_angle = - 180/np.pi * (-np.pi/4 + environment.simulation_data["pm_space"].bodies[3].angle - environment.simulation_data["pm_space"].bodies[1].angle)
    
    #arbitrary acceleration (units of change in speed per tick)
    acceleration = 1
    
    #target speeds but with directions defined aswell
    signs = np.sign(np.array([action[0], action[2]]) - np.array([leg_angle, torso_angle])) * (np.pi / 180) * [action[1], action[3]]
    
    #no. of timesteps needed to slow down from a given speed (for leg and torso)
    time_needed_l = abs(environment.simulation_data["speeds"][0])/acceleration
    time_needed_t = abs(environment.simulation_data["speeds"][1])/acceleration


    #if target is positive (anti-clockwise)
    if action[0] > leg_angle:
        #if the leg wont reach its target within the time needed to slow to a stop
        if action[0] >= leg_angle + (time_needed_l * (180/np.pi) * environment.simulation_data["speeds"][0]*environment.simulation_data["setup"]["step_length"]*environment.simulation_data["setup"]["sim_steps_per_decision"]):
            #if the speed is at the target speed - pass
            if environment.simulation_data["speeds"][0] == signs[0]:
                pass
            #if the speed is lower than the target speed - accelerate
            elif environment.simulation_data["speeds"][0] < signs[0]:
                environment.simulation_data["speeds"][0] += acceleration
            #if the speed is greater than the target speed - decelerate
            elif environment.simulation_data["speeds"][0] > signs[0]:
                environment.simulation_data["speeds"][0] -= acceleration
        #if the leg will reach its target within the time it takes to stop
        else:
            #if the speed is positive - decelerate
            if environment.simulation_data["speeds"][0] > 0.1:
                  environment.simulation_data["speeds"][0] -= acceleration
            #if the speed is negative - accelerate
            elif environment.simulation_data["speeds"][0] < -0.1:
                  environment.simulation_data["speeds"][0] += acceleration
            #if the speed is close to 0 set it to 0 to avoid random jitters
            else:
                environment.simulation_data["speeds"][0] = 0
    
    #same idea as above but for negative (clockwise) motion
    elif action[0] < leg_angle:
        if action[0] <= leg_angle + (time_needed_l * (180/np.pi) * environment.simulation_data["speeds"][0]*environment.simulation_data["setup"]["step_length"]*environment.simulation_data["setup"]["sim_steps_per_decision"]):
            if environment.simulation_data["speeds"][0] == signs[0]:
                pass
            elif environment.simulation_data["speeds"][0] < signs[0]:
                environment.simulation_data["speeds"][0] += acceleration
            elif environment.simulation_data["speeds"][0] > signs[0]:
                environment.simulation_data["speeds"][0] -= acceleration
        else:
            if environment.simulation_data["speeds"][0] > 0.1:
                  environment.simulation_data["speeds"][0] -= acceleration
            elif environment.simulation_data["speeds"][0] < -0.1:
                  environment.simulation_data["speeds"][0] += acceleration
            else:
                environment.simulation_data["speeds"][0] = 0
                
    #if the target speed is 0 accelerate towards 0
    else:
        if environment.simulation_data["speeds"][0] == signs[0]:
            pass
        elif environment.simulation_data["speeds"][0] < signs[0]:
            environment.simulation_data["speeds"][0] += acceleration
        elif environment.simulation_data["speeds"][0] > signs[0]:
            environment.simulation_data["speeds"][0] -= acceleration
    
    #add a motor to the knee joint with the current speed as defined above
    add_motor_l(simulation_data, environment.simulation_data["speeds"][0])
    
    #same as above but for the torso rather than the leg
    if action[2] > torso_angle:
        if action[2] >= torso_angle + (time_needed_t * (180/np.pi) * environment.simulation_data["speeds"][1]*environment.simulation_data["setup"]["step_length"]*environment.simulation_data["setup"]["sim_steps_per_decision"]):
            if environment.simulation_data["speeds"][1] == signs[1]:
                pass
            elif environment.simulation_data["speeds"][1] < signs[1]:
                environment.simulation_data["speeds"][1] += acceleration
            elif environment.simulation_data["speeds"][1] > signs[1]:
                environment.simulation_data["speeds"][1] -= acceleration
        else:
            if environment.simulation_data["speeds"][1] > 0.1:
                  environment.simulation_data["speeds"][1] -= acceleration
            elif environment.simulation_data["speeds"][1] < -0.1:
                  environment.simulation_data["speeds"][1] += acceleration
            else:
                environment.simulation_data["speeds"][1] = 0
    
    elif action[2] < torso_angle:
        if action[2] <= torso_angle + (time_needed_t * (180/np.pi) * environment.simulation_data["speeds"][1]*environment.simulation_data["setup"]["step_length"]*environment.simulation_data["setup"]["sim_steps_per_decision"]):
            if environment.simulation_data["speeds"][1] == signs[1]:
                pass
            elif environment.simulation_data["speeds"][1] < signs[1]:
                environment.simulation_data["speeds"][1] += acceleration
            elif environment.simulation_data["speeds"][1] > signs[1]:
                environment.simulation_data["speeds"][1] -= acceleration
        else:
            if environment.simulation_data["speeds"][1] > 0.1:
                  environment.simulation_data["speeds"][1] -= acceleration
            elif environment.simulation_data["speeds"][1] < -0.1:
                  environment.simulation_data["speeds"][1] += acceleration
            else:
                environment.simulation_data["speeds"][1] = 0
            
    else:
        if environment.simulation_data["speeds"][1] == signs[1]:
            pass
        elif environment.simulation_data["speeds"][1] < signs[1]:
            environment.simulation_data["speeds"][1] += acceleration
        elif environment.simulation_data["speeds"][1] > signs[1]:
            environment.simulation_data["speeds"][1] -= acceleration
        
    add_motor_t(simulation_data, environment.simulation_data["speeds"][1])
    
    return simulation_data

def add_motor_l(simulation_data, speed):
    'adds a motor to the knee with a given speed'
    simulation_data["motors"]["leg"] = Simplemotor(simulation_data["bodies"]["swing"].body,
                                                     simulation_data["bodies"]["leg"].body, speed,
                                                     simulation_data["pm_space"])

    
def add_motor_t(simulation_data, speed):
    'adds a motor to the torso with a given speed'
    simulation_data["motors"]["torso"] = Simplemotor(simulation_data["bodies"]["swing"].body,
                                                     simulation_data["bodies"]["torso"].body, speed,
                                                     simulation_data["pm_space"])



# ---------------------------------------------------------------------------------------------------------------------
# manual actions from keypresses:

def get_action(keytouple):
    'for manual control of the simulation (switch ML file to main() at the end)'

    if keytouple[pygame.K_l]:
        'leg clockwise'
        leg_action = np.array([1, 1, 0, 0])

    elif keytouple[pygame.K_j]:
        'leg anti-clockwise'
        leg_action = np.array([-1, 1, 0, 0])

    else:
        'for no input the leg goes to the minimum speed (-1, which is then redifined to 0 in the perform action function'
        leg_action = np.array([0, -1, 0, 0])
        
    if keytouple[pygame.K_d]:
        'torso clockwise'
        torso_action = np.array([0, 0, 1, 1])

    elif keytouple[pygame.K_a]:
        'torso anti-clockwise'
        torso_action = np.array([0, 0, -1, 1])

    else:
        'torso to 0 - same redifing as leg'
        torso_action = np.array([0, 0, 0, -1])
    
    'returns the total action with leg and torso components'
    return leg_action + torso_action
