"""
This module provides utility classes and functions for setting up and running simulations of 
mass-spring-damper systems, as well as for controlling such systems. It includes dataclasses 
for easily organizing and passing parameters related to the system, controller, and simulation 
settings. Additionally, it offers functionality to export simulation results to CSV files for 
further analysis or visualization.

Classes:
    MassSpringDamperSystemParams: Holds the parameters for initializing a mass-spring-damper system.
    ControllerParams: Contains parameters for initializing a controller, with defaults for optional gains.
    SimulationParams: Encapsulates parameters for running simulations, including setpoints and time steps.

Functions:
    output_simulation_to_csv: Exports simulation time steps, positions, and velocities to a CSV file.

"""

# Standard library imports
from dataclasses import dataclass
from typing import Tuple

# Third party imports
import pandas as pd
import numpy as np

# Local imports


@dataclass
class MassSpringDamperSystemParams:
    """Dataclass for the initialization parameters of the MassSpringDamper class.
        
    Attributes:
        m (float): Mass of the system in kilograms.
        k (float): Spring constant in Newtons per meter.
        b (float): Damping coefficient in Newton-seconds per meter.
    """
    m: float
    k: float
    b: float


@dataclass
class ControllerParams:
    """Dataclass for the initialization of the BaseController class.

    Attributes:
        kp (float): Proportional gain.
        ki (float): Integral gain. Default is 0.0 for controllers that don't use it.
        kd (float): Derivative gain. Default is 0.0 for controllers that don't use it.
        dt (float): Time step delta. Default is 0.0 for controllers that don't use it.
    """
    kp: float
    ki: float = 0.0
    kd: float = 0.0
    dt: float = 0.0


@dataclass
class SimulationParams:
    """Dataclass for arguments to the `simulate` method of the `Simulation` class.

    Args:
        setpoint (float): The desired setpoint for the system's position.
        initial_conditions (tuple[float, float]): The initial position and velocity of the system (position, velocity).
        duration (float): The total duration of the simulation in seconds.
        dt_system (float): The time step for the system simulation.
        dt_controller (float): The time step for the controller updates.

    """
    setpoint: float
    initial_conditions: tuple[float, float]
    duration: float
    dt_system: float
    dt_controller: float


def output_simulation_to_csv(times: np.ndarray, positions: np.ndarray, velocities: np.ndarray, setpoints: np.ndarray, filename: str) -> None:
    """Utility function for exporting the time steps and associated positions & velocities from a simulation to a CSV.

    Args:
        times (np.ndarray): 1D array of simulation time steps.
        positions (np.ndarray): 1D array of simulation positions for each time step.
        velocities (np.ndarray): 1D array of simulation velocities for each time step.
        setpoints (np.ndarray): 1D array of the setpoint for the simulation for each timestep. All elements are equal.
        filename (str): the filename to export to. Should end in ".csv".

    Raises:
        TypeError: if any argument is not of the expected type.
        ValueError: if the given name for the file does not end in ".csv".
        IndexError: if the 3 arrays are not of the same size.
    
    """
    # Argument checking
    # Type checking
    if not isinstance(times, np.ndarray):
        raise TypeError(f"arg 'times' must be of type np.ndarray, not {type(times)}")
    if not isinstance(positions, np.ndarray):
        raise TypeError(f"arg 'positions' must be of type np.ndarray, not {type(positions)}")
    if not isinstance(velocities, np.ndarray):
        raise TypeError(f"arg 'velocities' must be of type np.ndarray, not {type(velocities)}")
    if not isinstance(setpoints, np.ndarray):
        raise TypeError(f"arg 'setpoints' must be of type np.ndarray, not {type(setpoints)}")
    if not isinstance(filename, str):
        raise TypeError(f"arg 'filename' must be of type str, not {type(filename)}")
    # Length equality checking
    if not (len(times) == len(positions) == len(velocities)):
        raise IndexError(f"args 'times' (size {len(times)}), 'positions' (size {len(positions)}), 'velocities' (size {len(velocities)}) must all be of same size")
    # Filename checking
    if not filename.endswith(".csv"):
        raise ValueError(f"filename '{filename}' must end in '.csv'")

    df = pd.DataFrame({
        "time": times,
        "position": positions,
        "velocity": velocities
    })
    df.to_csv(filename, index=False)


#####################

# Baseline params for individual tests
p_controller_params = ControllerParams(kp=10.0, ki=0.0, kd=0.0)
pi_controller_params = ControllerParams(kp=10.0, ki=2.8, kd=0.0)
pd_controller_params = ControllerParams(kp=400.0, ki=0.0, kd=50.0)
pid_controller_params = ControllerParams(kp=350.0, ki=300.0, kd=50.0)
msd_params = MassSpringDamperSystemParams(m=1.0, k=10.0, b=0.5)
simulation_params = SimulationParams(
    setpoint=1.0,
    initial_conditions=(0.0, 0.0),
    duration=60.0, # usually 60
    dt_system=0.01,      # doesn't actually get used but has to be set
    dt_controller=0.01,  # doesn't actually get used but has to be set
)
