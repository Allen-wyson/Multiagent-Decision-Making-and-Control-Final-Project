import numpy as np
from car_dynamics import bicycle_dynamics_discrete
from frenet import get_frenet_coords

class Car:
    def __init__(self, initial_state, v_max, a_max, a_lat_max, max_steering_angle=np.pi/2, dt=0.1, L=2.5):
        """
        init_state: [x, y, theta, v]
        """
        self.state = np.array(initial_state, dtype=float)
        self.control_inputs = np.array([0.0, 0.0])  
        self.dt = dt
        self.L = L
        self.history = [self.state.copy()]
        self.v_max = v_max
        self.a_max =  a_max
        self.a_lat_max = a_lat_max
        self.max_steering_angle = max_steering_angle

    def set_control(self, a, delta):
        """Set current control inputs."""
        self.control_input = np.array([a, delta])

    def update(self):
        """Advance the state using bicycle dynamics."""
        self.state = bicycle_dynamics_discrete(self.state, self.control_input, self.dt, self.L)
        self.history.append(self.state.copy())

    def get_state(self):
        """Return the current state."""
        return self.state

    def get_frenet_coords(self, centerline, headings):
        """Return current (s, d) in Frenet frame."""
        x, y = self.state[0], self.state[1]
        return get_frenet_coords(x, y, centerline, headings)
