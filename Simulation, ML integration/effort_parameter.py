import numpy as np

def holding(self, limb, centre, width):
    return ((1-(((self.simulation_data["pm_space"].bodies[limb].angle - self.simulation_data["pm_space"].bodies[1].angle) - centre) / width))**4) * 90*abs(np.sin(self.simulation_data["pm_space"].bodies[limb].angle))

def get_effort(self, leg_acc, torso_acc,k4 = 1,k5 = 1,k6 = 1,k7 = 1):
    leg_hold = holding(self, 2, 0, 90)
    torso_hold = holding(self, 3, 22.5, 135)
    return k4*leg_acc + k5*leg_hold + k6*torso_acc + k7*torso_hold
