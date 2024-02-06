"""
This module defines a Simulation class for running simulations of a mass-spring-damper system
controlled by a feedback mechanism. It integrates a given mass-spring-damper model and a controller
to simulate the system's response over time to achieve a desired setpoint. The simulation includes
calculations for system dynamics and control actions at specified time intervals.
"""

# Standard library imports
from typing import Tuple

# Third party imports
import numpy as np

# Local imports
from mass_spring_damper_simulation.controller import BaseController
from mass_spring_damper_simulation.mass_spring_damper import MassSpringDamper


class Simulation:
    """A class to simulate a mass-spring-damper system using a specified controller.
    
    This class integrates a mass-spring-damper system model with a control strategy to simulate the
    dynamic response of the system over time. It provides a method to run simulations with given
    initial conditions, setpoints, and simulation parameters.
    
    Attributes:
        system (MassSpringDamper): The mass-spring-damper system to be simulated.
        controller (BaseController): The controller used to regulate the system.
        time (float): The current time of the simulation.
    """
    
    def __init__(self, system: MassSpringDamper, controller: BaseController):
        """
        Initialize the Simulation class with a mass-spring-damper system and a PID controller.

        Args:
            system (MassSpringDamper): The mass-spring-damper system to simulate.
            controller (BaseController): The controller for the system.
        """
        self.system: MassSpringDamper = system
        self.controller: BaseController = controller
        self.time: float = 0.0

    def simulate(self, setpoint: float, initial_conditions: Tuple[float, float], duration: float, dt_system: float, dt_controller: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Simulates the mass-spring-damper system under the control of the controller.

        Args:
            setpoint (float): The desired setpoint for the system's position.
            initial_conditions (tuple[float, float]): The initial position and velocity of the system (position, velocity).
            duration (float): The total duration of the simulation in seconds.
            dt_system (float): The time step for the system simulation.
            dt_controller (float): The time step for the controller updates.

        Returns:
            tuple[np.ndarray, np.ndarray, np.ndarray]: A tuple containing the time steps, positions, and velocities
            of the system throughout the simulation. The first array is the time steps, the second is the positions,
            and the third is the velocities.
        """
        # Calculate the number of steps for the system and the controller
        n_system: int = int(duration / dt_system) + 1
        
        # Initialize state arrays for system
        x: np.ndarray = np.zeros(n_system)
        v: np.ndarray = np.zeros(n_system)
        x[0], v[0] = initial_conditions
        
        # Initialize control force array
        control_force: np.ndarray = np.zeros(n_system)
        
        # Initialize controller time and index
        controller_time: float = 0.0
        
        for i in range(n_system - 1):
            # Time for the current step
            current_time: float = i * dt_system
            
            # Update the controller at its own frequency
            if current_time >= controller_time:
                control_force[i] = self.controller.compute_control(setpoint, x[i])
                controller_time += dt_controller
            
            # Use the last computed control force for system dynamics update
            a: float = self.system.compute_acceleration(x[i], v[i], control_force[i])
            v[i + 1] = v[i] + a * dt_system
            x[i + 1] = x[i] + v[i] * dt_system
        
        return np.arange(0, duration + dt_system, dt_system), x, v
