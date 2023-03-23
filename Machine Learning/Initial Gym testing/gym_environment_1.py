import pygame
import numpy as np
import pymunk
import gym
from pymunk.pygame_util import DrawOptions
import time

window_width, window_height = 1000, 500
rotation_max, acceleration_max = 0.08, 0.5


class CustomEnv(gym.Env):
    def __init__(self, env_config={}):
        self.observation_space = gym.spaces.Box([-np.pi,np.pi],[],(2,),np.float32)
        self.action_space = gym.spaces.Box(0,1,(1,),np.uint8)

        self.motor_active = False
        self.timer = 0
        self.duration = 40

        self.pm_space = pymunk.Space()
        self.motor_rate = 3
        self.pivot_body = self.pm_space.static_body

        self.body = pymunk.Body()
        self.body.position = (400, 200)

        self.shape = pymunk.Segment(self.body, (0, -100), (0, 100), 10)
        self.shape.density = 1
        self.pm_space.add(self.body, self.shape)

        self.pivot_joint = pymunk.constraints.PinJoint(self.pivot_body, self.body, (400, 100), (0, -100))
        self.pm_space.add(self.pivot_joint)

    def init_render(self):
        import pygame
        pygame.init()
        self.window = pygame.display.set_mode((window_width, window_height))
        self.pm_space.gravity = 0, 981
        self.options = DrawOptions(self.window)
        self.clock = pygame.time.Clock()

    def reset(self):
        # reset the environment to initial state
        return observation

    def step(self, action=np.zeros((1), dtype=np.single)):
        self.handle_events(action)
        self.pm_space.step(1/1000)
        #print(self.pm_space.bodies[0].position)
        observation, reward, done, info = 0., 0., False, {}
        return observation, reward, done, info

    def render(self):
        #pass
        self.window.fill((255, 255, 255))
        self.pm_space.debug_draw(self.options)
        pygame.display.update()

    def handle_events(self,action):
        if self.motor_active:
            if self.timer <= self.duration:
                self.timer += 1
            else:
                self.motor_active = False
                self.remove_motor()
        else:
            if action[0]:
                self.add_motor()
                self.timer = 0
                self.motor_active = True

    def add_motor(self):
        self.motor = pymunk.SimpleMotor(self.pivot_body,self.body,self.motor_rate)
        self.pm_space.add(self.motor)

    def remove_motor(self):
        self.pm_space.remove(self.motor)


def pressed_to_action(keytouple):
    impulse = False
    if keytouple[pygame.K_SPACE]:  # back
        impulse = True
    return np.array([impulse])


environment = CustomEnv()
environment.init_render()
x = 0
run = True
start = time.time()
while run:
    # set game speed to 100 fps
    #environment.clock.tick(1000)
    # ─── CONTROLS ───────────────────────────────────────────────────────────────────
    # end while-loop when window is closed
    get_event = pygame.event.get()
    for event in get_event:
        if event.type == pygame.QUIT:
            run = False
    # get pressed keys, generate action
    get_pressed = pygame.key.get_pressed()
    action = pressed_to_action(get_pressed)
    # calculate one step
    environment.step(action)
    # render current state
    environment.render()

    x+= 1
    if x == 100000:
        run = False
end = time.time()
print("RATE:",round(x/(end-start),3))
pygame.quit()
