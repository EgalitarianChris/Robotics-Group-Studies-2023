'''get_angle does not work, need to fix to limit the torso movement like with leg'''

import pymunk, sys
from pymunk.pygame_util import *
from pymunk.vec2d import Vec2d
import pygame
from pygame.locals import *
import numpy as np
from PIL import Image
from pymunk.pygame_util import DrawOptions


size = 800, 800
display = pygame.display.set_mode((size))
options = DrawOptions(display)
clock = pygame.time.Clock()
space = pymunk.Space()
space.gravity = 0, 981
b0 = space.static_body
b1 = space.static_body
FPS = 120

def convert_coordinates(point):#pymunk and pygame use different coordinate systems for no good reason
    return int(point[0]), int(800-point[1])

def get_theta(x_h, x_1, y_h, y_1):#angle of top hinge
    return np.arctan2(x_1 - x_h, y_1 - y_h)

def get_phi(x1, x2, y1, y2, theta):# angle between segment and swing
    return np.arctan2(x2 - x1, y2 - y1) - theta

def get_iota(x1, x2, y1, y2, theta, phi):# angle of leg from swing
    return np.arctan2(x2 -x1, y2 - y1) - theta - phi

def get_angle(x6, x7, y6, y7, theta, phi):# angle of torso from swing but is worng atm
    return np.arctan2(x7 - x6, y7 - y6)-theta-phi

    
class Segment:
    def __init__(self, p0, a, b, radius=10, center_of_gravity = (0,0), density=0.01):
        self.body = pymunk.Body()
        self.body.position = p0
        self.radius = radius
        self.a = a
        self.b = b
        self.body.center_of_gravity = center_of_gravity
        self.shape = pymunk.Segment(self.body, self.a, self.b, radius)
        self.shape.density = density
        self.shape.elasticity = 0
        self.shape.filter = pymunk.ShapeFilter(group=1)
        self.shape.color = (0, 255, 0, 0)
        space.add(self.body, self.shape)

class Leg:
    def __init__(self, p0, a, b, c, d,radius=10, center_of_gravity = (0,0), density=0.01):
        self.body = pymunk.Body()
        self.body.position = p0
        self.radius = radius
        self.a = a
        self.b = b
        self.c = c
        self.d = d
        self.body.center_of_gravity = center_of_gravity
        self.leg= pymunk.Segment(self.body, self.a, self.b , radius=radius)
        self.leg.filter = pymunk.ShapeFilter(group = 1)
        self.leg.density = density
        self.leg.color = (0, 255, 0, 0)
        self.foot= pymunk.Segment(self.body, self.c, self.d, radius=radius)
        self.foot.filter = pymunk.ShapeFilter(group = 1)
        self.foot.density = density
        self.foot.filter = pymunk.ShapeFilter(group=1)
        self.foot.color = (0, 255, 0, 0)
        space.add(self.body, self.leg, self.foot)
        
        
        
class Simplemotor:#is added and removed at diffrent points to move a joint at a constant speed
#arbitrary 0-5 for min-max angular rotation speed, need to convert joint rotations speed into fraction of this.
    def __init__(self, b, b2, rate=5, switch="off"):
        self.rate = rate
        self.b = b
        self.b2 = b2
        self.simplemotor = pymunk.SimpleMotor(self.b, self.b2, self.rate)
        space.add(self.simplemotor)
    def remove(self):
        space.remove(self.simplemotor)

class RotaryLimitJoint:#stops swing moving out of control area i think
    def __init__(self, b, b2, min, max, collide=True):
        joint = pymunk.constraints.RotaryLimitJoint(b, b2, min, max)
        joint.collide_bodies = collide
        space.add(joint)

class PivotJoint:#seems unused
    def __init__(self, b, b2, a=(0, 0), a2=(0, 0), collide=True):
        joint = pymunk.constraints.PinJoint(b, b2, a, a2)
        joint.collide_bodies = collide
        space.add(joint)

class PinJoint:
    def __init__(self, b, b2, a=(0, 0), a2=(0, 0)):
        joint = pymunk.constraints.PinJoint(b, b2, a, a2)
        space.add(joint)

class Swing_body:
    def __init__(self,p0, vx1,vy1,vx2,vy2,vx3,vy3, a, b, radius=10, center_of_gravity = (0,0), density=0.05):
        self.body = pymunk.Body()
        self.body.position = p0
        s1 = pymunk.Segment(self.body, vx1, vy1 , radius) #bar
        s1.filter = pymunk.ShapeFilter(group = 1)
        s1.density = density
        s2 = pymunk.Segment(self.body, vx2, vy2, radius)#vertical
        s2.filter = pymunk.ShapeFilter(group = 1)
        s2.density = density
        s3 = pymunk.Segment(self.body, vx3,vy3, radius)#base
        s3.filter = pymunk.ShapeFilter(group = 1)
        s3.density = density
        s4 = pymunk.Segment(self.body, a, b, radius)#torso
        s4.filter = pymunk.ShapeFilter(group = 1)
        s4.density = density
        space.add(self.body, s1,s2,s3)

class Torso:
    def __init__(self, p0, a, b, radius=10, center_of_gravity = (0,0), density=0.01):
        self.body = pymunk.Body()
        self.body.position = p0
        self.radius = radius
        self.a = a
        self.b = b
        self.body.center_of_gravity = center_of_gravity
        self.leg= pymunk.Segment(self.body, self.a, self.b , radius=radius)        
        self.leg.filter = pymunk.ShapeFilter(group = 1)
        self.leg.density = density
        self.leg.color = (255, 0, 0, 0)
        space.add(self.body, self.leg)

def angle_reached(theta, high_score):#seems to be useless, works out the max angle reached by the simulation
    if len(high_score) == 0:
        high_score.append(theta)
    elif high_score[0] < abs(theta):
        high_score[0] = abs(theta)
    highest_score = high_score[0]
    return high_score

'''random coordinates for the different body parts etc'''
hinge_point1 = (0, -100) # seg 1
hinge_point2 = (0, 100)
swing_body = (400, 625)
swing_top1 = (30, -25)
swing_top2 = (0, -25)
swing_mid1 = (0, -25)
swing_mid2 = (0, 25)
swing_bottom1 = (-20, 25)
swing_bottom2 = (20, 25)
hinge_point3 = (0, -30) # seg 2
hinge_point4 = (0, 30)


torsoup = (-10,-20)
torsodown = (10,20)
"Pymunk Bodies"
torso = Torso((370,630),torsodown,torsoup, density= 0.05)

segment = Segment((400 , 500), hinge_point1 , hinge_point2)
leg = Leg((420,680), hinge_point3, hinge_point4, (0,30), (15,30), density= 0.05)
swing = Swing_body(swing_body, swing_top1,swing_top2, swing_mid1, swing_mid2, swing_bottom1, swing_bottom2,(0,25),(-20,-25))

PinJoint(swing.body, torso.body, swing_bottom1, (10,20))#second needs fixing to first

PinJoint(swing.body, leg.body, swing_bottom2, hinge_point3)
PinJoint(segment.body, swing.body, hinge_point2, swing_mid1)
PinJoint(b0, segment.body, (400,400), hinge_point1)
rotation_lim = RotaryLimitJoint(segment.body,swing.body , -np.pi/4, np.pi/4)

'''
x1,y1 = segment center[0],[1]
xh,yh = fixed point for upper hinge
x2,y2 = joint 2 pos[0],[1]
x3,y3 = swing center[0],[1]
x4,y4  = joint 3 pos[0],[1]
x5,y5 = leg center[0],[1]

theta = angle from segment center to joint 1
phi = angle from swing center to joint 2
iota = angle from leg to joint 3

x6,y6 = joint 4 pos[0],[1]
x7,y7 = torso center'''

def main():
    pygame.display.set_caption("Double pendulum interactive Simulation")
    high_score = []
    x = "const motor"
    x_ = "const motor"
    constant_motor1 = Simplemotor(swing.body, leg.body, 0)
    constant_motor2 = Simplemotor(swing.body, torso.body, 0)
    while True:
        xh, yh = (400,400)
        x1, y1 = segment.body.position[0], segment.body.position[1]
        theta = get_theta(xh, x1, yh, y1)
        x2, y2 = segment.body.position[0] + 100*np.sin(theta) , segment.body.position[1] + 100*np.cos(theta) 
        x3, y3 = swing.body.position[0], swing.body.position[1]
        phi = get_phi(x2, x3, y2, y3, theta)
        x4, y4 = swing.body.position[0] + 25*np.sin(theta+phi) + 20*np.cos(theta+phi), swing.body.position[1] + 25*np.cos(theta+phi) - 20*np.sin(theta+phi) 
        x5, y5 = leg.body.position[0], leg.body.position[1]
        iota = get_iota(x4, x5, y4, y5, theta, phi)
        
        x6, y6 = swing.body.position[0] - 25*np.sin(theta+phi) - 20*np.cos(theta+phi), swing.body.position[1] + 25*np.cos(theta+phi) + 20*np.sin(theta+phi) 
        x7, y7 = torso.body.position[0], torso.body.position[1]
        angle = get_angle(x6, x7, y6, y7, theta, phi)
        
        # pygame.draw.circle(display, (0,0,255), (x6,y6), 20)
        
        const_angvel = swing.body.angular_velocity
        for event in pygame.event.get():  # checking for user input
            if event.type == pygame.QUIT:
                print(f"Highest angle reached was:{np.rad2deg(high_score)}")
                pygame.quit()
                sys.exit()
            if event.type == KEYDOWN:
                if event.key == K_RIGHT:
                    if iota < np.pi/2 and x == "const motor":
                        constant_motor1.remove()
                        x = "positive motor"
                        pos_motor = Simplemotor(swing.body, leg.body, 5)
                    elif iota < np.pi/2 and x == "negetive motor":
                        neg_motor.remove()
                        x = "positive motor"
                        pos_motor = Simplemotor(swing.body, leg.body, 5)
                elif event.key == K_LEFT:
                    if iota > 0 and x == "const motor":
                        constant_motor1.remove()
                        x = "negetive motor"
                        neg_motor = Simplemotor(swing.body, leg.body, -5)
                    elif iota > 0 and x == "positive motor":
                        pos_motor.remove()
                        x = "negetive motor"
                        neg_motor = Simplemotor(swing.body, leg.body, -5)
                if event.key == K_UP:
                    if angle > -np.pi/2 and x_ == "const motor":
                        constant_motor2.remove()
                        x_ = "positive motor"
                        pos_motor2 = Simplemotor(swing.body, torso.body, 1)
                    elif angle > -np.pi/2 and x_ == "negetive motor":
                        neg_motor2.remove()
                        x_ = "positive motor"
                        pos_motor2 = Simplemotor(swing.body, torso.body, 1)
                elif event.key == K_DOWN:
                    if angle < 0 and x_ == "const motor":
                        constant_motor2.remove()
                        x_ = "negetive motor"
                        neg_motor2 = Simplemotor(swing.body, torso.body, -1)
                    elif angle < 0 and x_ == "positive motor":
                        pos_motor2.remove()
                        x_ = "negetive motor"
                        neg_motor2 = Simplemotor(swing.body, torso.body, -1)
            
                        
#limits for legs angles---------------------------------------------
        if iota >= (np.pi/2-0.1) and x == "positive motor":
            pos_motor.remove()
            constant_motor1 = Simplemotor(swing.body, leg.body, 0)
            x = "const motor"
        if iota <= (-np.pi/6 + 0.08) and x == "negetive motor":
            neg_motor.remove()
            constant_motor1 = Simplemotor(swing.body, leg.body, 0)
            x = "const motor"

#limits for torso (unfinished)----------------------------------------------
        # if angle >= -(np.pi/2-0.1) and x_ == "positive motor":
        #     pos_motor2.remove()
        #     constant_motor2 = Simplemotor(swing.body, leg.body, 0)
        #     x = "const motor"
        # if angle <= (-np.pi/6 + 0.08) and x_ == "negetive motor":
        #     neg_motor2.remove()
        #     constant_motor2 = Simplemotor(swing.body, leg.body, 0)
        #     x = "const motor"


        # print(iota)
        high_score = angle_reached(theta, high_score)
        display.fill((255, 255, 255))
        space.debug_draw(options)
        pygame.display.update() #comment out to remove gui
        clock.tick(FPS)  # limiting frames per second to 120
        space.step(1/FPS)#this determines fhow much "time" ellapses between each run of the while loop
        # changing this could change the polling rate for the machine learning
        print(len(space.constraints))
        print(x)
        # print('angle = ', angle/np.pi*180)
        print('iota = ', iota/np.pi*180)
        # print('x4=',x4)
        # print('x6=',x6)
        # print('y4=',y4)
        # print('y6=',y6)
        # print('phi=',phi)
    pygame.quit()
    quit()
        

main()
pygame.quit()

def run(config_path):
    pass

if __name__ == "__main__":
    local_dir = os.path.dirname(__file__) # gives us path to the directory of this script
    config_path = os.path.join(lacal_dir, "cofig-feedfor")
    