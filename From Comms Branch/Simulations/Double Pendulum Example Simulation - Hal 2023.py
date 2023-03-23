"""
This is an example of a Pymunk/Pygame simulation for a double pendulum coded by Hal.
Please message me on Discord if you have any questions about the code since I think I understand it all.

The class "Pendulum" conatins all the information that configures the simulation and would allow it to be modified for a swing.

For examples of building out the full shape of the swing, take a look at the 2022 github at simulations 3 and 4.

NOTE - THIS IS A VERY POOR SIUMULATION. THE FRAMERATE LIMITER OF 120FPS THAT COUPLES PYGAME AND PYMUNK HUGELY LIMITS THE TIMESTEP SIZE AND ACCURACY OF THE SIMULATION
- AN ASYNCHRONOUS / NON-LIMITED FORM OF THIS SIMULATION SHOULD BE USED WITH SUBSTANTIALLY SMALLER TIMESTEPS FOR THE FINAL SIMULATION (realtime simulation is not neccessary)
"""

# import statements
import pygame
import pymunk
import numpy as np
from pymunk.pygame_util import DrawOptions

# double pendulum class - contains all of the properties of the double pendulum being simulated
class Pendulum:

    def __init__(self,space,b0,pivot_position,pendulum_1_length,pendulum_2_length):

        # Motors: ------------------------------------------------------------------------------------------
        self.motor_1_rate = 3
        self.motor_2_rate = 3

        # Pendulum 1: --------------------------------------------------------------------------------------

        # upper pendulum rod extent
        self.pendulum_1_start_pos = (0, -pendulum_1_length/2)  # top of pendulum 1 (offset from centre position)
        self.pendulum_1_end_pos = (0, pendulum_1_length/2)  # bottom of pendulum 1 (offset from centre position)

        self.pendulum_1_body = pymunk.Body() # pymunk body instance
        self.pendulum_1_body.position = (pivot_position[0],pivot_position[1]+pendulum_1_length/2) # body position (centre) within simulation (800x800)

        # define pendulum shape using rod segment from start to end pos with thickness 10
        self.pendulum_1_shape = pymunk.Segment(self.pendulum_1_body, self.pendulum_1_start_pos, self.pendulum_1_end_pos, 10)

        self.pendulum_1_shape.density = 0.1
        self.pendulum_1_shape.filter = pymunk.ShapeFilter(group=1)

        space.add(self.pendulum_1_body, self.pendulum_1_shape) # add the pymunk body to the simulation space with the segment shape

        # Pendulum 2: -------------------------------------------------------------------------------------

        # lower pendulum rod extent
        self.pendulum_2_start_pos = (0, -pendulum_2_length/2)  # top of pendulum 2 (offset from centre position)
        self.pendulum_2_end_pos = (0, pendulum_2_length/2)  # bottom of pendulum 2 (offset from centre position)

        self.pendulum_2_body = pymunk.Body()  # pymunk body instance
        self.pendulum_2_body.position = (pivot_position[0],pivot_position[1]+pendulum_1_length+pendulum_2_length/2)

        # define pendulum shape using rod segment from start to end pos with thickness 10
        self.pendulum_2_shape = pymunk.Segment(self.pendulum_2_body, self.pendulum_2_start_pos, self.pendulum_2_end_pos,
                                               10)

        self.pendulum_2_shape.density = 0.1
        self.pendulum_2_shape.filter = pymunk.ShapeFilter(group=1)

        space.add(self.pendulum_2_body,self.pendulum_2_shape)  # add the pymunk body to the simulation space with the segment shape

        # constraints (pinjoints):-------------------------------------------------------------------------

        # create joint between pivot (arg 1) and upper pendulum (arg 2) (joint is located on the pivot at its position (arg 3), located on the upper pendulum at its start pos (arg 4))
        self.pinjoint_1 = pymunk.constraints.PinJoint(b0, self.pendulum_1_body, pivot_position, (0, -pendulum_1_length/2))
        space.add(self.pinjoint_1)

        self.pinjoint_2 = pymunk.constraints.PinJoint(self.pendulum_1_body, self.pendulum_2_body, (0, pendulum_1_length/2), (0, -pendulum_1_length/2))
        space.add(self.pinjoint_2)

    # basic motor add/ motor removal functions - these should be used in another function to represent the way the robot's motors actually work
    def add_motor(self, motor_number, space):
        if motor_number == 2:
            self.motor_2 = pymunk.SimpleMotor(self.pendulum_1_body,self.pendulum_2_body,self.motor_2_rate)
            space.add(self.motor_2)

    def remove_motor(self, motor_number, space):
        if motor_number == 2:
            space.remove(self.motor_2)


# setup function for pygame/pymunk instances
def setup():
    display = pygame.display.set_mode((800, 800)) # create pygame window
    pygame.display.set_caption("Double pendulum interactive Simulation") # apply a caption to the window
    options = DrawOptions(display) # get the options instance required for the Pymunk simulation state to be displayed on the pygame window
    clock = pygame.time.Clock() # clock instance used to limit the simulation/ framerate - should not be used on future implementations
    space = pymunk.Space() # creates the simulation "space" used by pymunk - essentially a container for the bodies to be simulated in. all simulated elements get added to this
    space.gravity = 0, 981
    b0 = space.static_body # the fixed pivot point attatched to the top of the system

    # (below) creates the simulation - this is where an instance of the Pendulum class object described above is created, passing the parameters: (space,b0,pivot_position,l1,l2)
    pymunk_pendulum_instance = Pendulum(space,b0,[400,100],200,200)

    return display, clock, space, b0, options, pymunk_pendulum_instance # return everything used by the game() function


# iterative loop for running and displaying the simulation
def game(display, clock, space, b0, FPS, options, pymunk_pendulum_instance):
    # Example motor used to start the pendulum's swing
    x= 0
    pymunk_pendulum_instance.add_motor(2, space)

    while True:
        # Example motor used to start the pendulum's swing
        x += 1
        if x == 120:
            pymunk_pendulum_instance.remove_motor(2, space)

        # checking for user input - stops pygame from crashing
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        display.fill((255, 255, 255)) # draw white background

        space.debug_draw(options) # draw the simulation state in pygame window
        pygame.display.update() # update the pygame display

        clock.tick(FPS) # limit simulation computation rate
        space.step(1 / FPS) # increase simulation time by timestep


def main():
    display, clock, space, b0, options, pymunk_pendulum_instance = setup() # setup simulation
    game(display, clock, space, b0, 120, options, pymunk_pendulum_instance) # run simulation
    pygame.quit() # quit simulation

main()
