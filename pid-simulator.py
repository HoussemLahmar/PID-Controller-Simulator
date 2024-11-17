import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from dataclasses import dataclass
import json
import time

@dataclass
class PIDController:
    Kp: float
    Ki: float
    Kd: float
    setpoint: float = 0
    
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()
    
    def compute(self, process_value, dt):
        error = self.setpoint - process_value
        
        # Proportional term
        P = self.Kp * error
        
        # Integral term
        self.integral += error * dt
        I = self.Ki * self.integral
        
        # Derivative term
        derivative = (error - self.prev_error) / dt
        D = self.Kd * derivative
        
        # Save error for next iteration
        self.prev_error = error
        
        return P + I + D, error

class Plant:
    """Base class for different plant models"""
    def __init__(self):
        pass
    
    def simulate(self, t, y, u):
        raise NotImplementedError

class DCMotor(Plant):
    def __init__(self, J=0.01, b=0.1, K=0.01, R=1, L=0.5):
        self.J = J  # moment of inertia
        self.b = b  # damping ratio
        self.K = K  # motor constant
        self.R = R  # resistance
        self.L = L  # inductance
    
    def simulate(self, t, y, u):
        # y = [angular_velocity, current]
        omega, i = y
        
        # State equations for DC motor
        domega = (self.K * i - self.b * omega) / self.J
        di = (-self.R * i - self.K * omega + u) / self.L
        
        return [domega, di]

class InvertedPendulum(Plant):
    def __init__(self, M=0.5, m=0.2, b=0.1, I=0.006, g=9.8, l=0.3):
        self.M = M  # cart mass
        self.m = m  # pendulum mass
        self.b = b  # friction coefficient
        self.I = I  # moment of inertia
        self.g = g  # gravity
        self.l = l  # pendulum length
    
    def simulate(self, t, y, u):
        # y = [x, x_dot, theta, theta_dot]
        x, x_dot, theta, theta_dot = y
        
        # Simplified equations of motion
        sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)
        
        # Denominator terms
        d = self.I * (self.M + self.m) + self.M * self.m * self.l**2 * sin_theta**2
        
        # State equations for inverted pendulum
        dx = x_dot
        dx_dot = (u - self.b * x_dot + self.m * self.l * theta_dot**2 * sin_theta - 
                  self.m * self.g * self.l * sin_theta * cos_theta) / d
        dtheta = theta_dot
        dtheta_dot = ((self.M + self.m) * self.g * self.l * sin_theta - 
                      self.m * self.l * cos_theta * u + 
                      self.b * self.l * cos_theta * x_dot) / d
        
        return [dx, dx_dot, dtheta, dtheta_dot]

class Simulator:
    def __init__(self, plant, controller, dt=0.01, simulation_time=10):
        self.plant = plant
        self.controller = controller
        self.dt = dt
        self.simulation_time = simulation_time
        self.time_points = np.arange(0, simulation_time, dt)
        self.results = None
    
    def run(self, initial_conditions, disturbance_time=None, disturbance_magnitude=0):
        results = []
        t_current = 0
        y = initial_conditions
        
        for t in self.time_points:
            # Apply control input
            if isinstance(self.plant, DCMotor):
                process_value = y[0]  # angular velocity
            elif isinstance(self.plant, InvertedPendulum):
                process_value = y[2]  # angle
            
            # Apply disturbance if specified
            disturbance = 0
            if disturbance_time and t >= disturbance_time:
                disturbance = disturbance_magnitude
            
            # Compute control signal
            u, error = self.controller.compute(process_value, self.dt)
            u += disturbance
            
            # Simulate one step
            y = odeint(lambda y, t: self.plant.simulate(t, y, u), y, [t, t + self.dt])[-1]
            
            # Store results
            results.append({
                'time': t,
                'input': u,
                'output': process_value,
                'error': error,
                'state': y.tolist()
            })
        
        self.results = results
        return results
    
    def plot_results(self):
        if not self.results:
            raise ValueError("No simulation results available. Run simulation first.")
        
        time = [r['time'] for r in self.results]
        output = [r['output'] for r in self.results]
        input_signal = [r['input'] for r in self.results]
        error = [r['error'] for r in self.results]
        
        plt.figure(figsize=(12, 8))
        
        # Plot system output
        plt.subplot(311)
        plt.plot(time, output, 'b-', label='Output')
        plt.plot(time, [self.controller.setpoint] * len(time), 'r--', label='Setpoint')
        plt.grid(True)
        plt.legend()
        plt.ylabel('Output')
        
        # Plot control input
        plt.subplot(312)
        plt.plot(time, input_signal, 'g-', label='Control Input')
        plt.grid(True)
        plt.legend()
        plt.ylabel('Control Input')
        
        # Plot error
        plt.subplot(313)
        plt.plot(time, error, 'r-', label='Error')
        plt.grid(True)
        plt.legend()
        plt.xlabel('Time (s)')
        plt.ylabel('Error')
        
        plt.tight_layout()
        plt.show()
    
    def export_data(self, filename):
        """Export simulation data to JSON file"""
        if not self.results:
            raise ValueError("No simulation results available. Run simulation first.")
        
        with open(filename, 'w') as f:
            json.dump(self.results, f)


