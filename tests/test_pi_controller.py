# Standard library imports

# Third party imports
import matplotlib.pyplot as plt
import numpy as np

# Local imports
from mass_spring_damper_simulation.mass_spring_damper import MassSpringDamper
from mass_spring_damper_simulation.simulation import Simulation
from mass_spring_damper_simulation.controller import PIController
from tests.utils import pi_controller_params, msd_params, simulation_params, output_simulation_to_csv


def test_pi_controller_equal_frequencies():

    # Frequencies
    dt_system = 0.001  # System frequency (1000 Hz)
    dt_controller = 0.001  # Controller frequency (1000 Hz)

    # Create instances of the system and the controller
    msd_system = MassSpringDamper(m=msd_params.m, k=msd_params.k, b=msd_params.b)
    pi_controller = PIController(kp=pi_controller_params.kp, ki=pi_controller_params.ki, dt=dt_controller)
    simulation = Simulation(msd_system, pi_controller)

    # Perform the simulation
    t, x, v = simulation.simulate(
        simulation_params.setpoint,
        simulation_params.initial_conditions,
        simulation_params.duration,
        dt_system,
        dt_controller,
    )

    # Plot the results
    plt.plot(t, x, label='Position')
    plt.plot(t, [simulation_params.setpoint]*len(t), label='Setpoint', linestyle='--')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.title(f'Mass-Spring-Damper System Simulation with PI Controller\nSystem: m={msd_params.m}, k={msd_params.k}, b={msd_params.b}, dt={dt_system}\nController: Kp={pi_controller.kp}, Ki={pi_controller.ki}, dt={dt_controller}')
    plt.legend()
    plt.grid(True)
    plt.savefig('msd_simulation_pi_control_equal_system_and_controller_freq.png', dpi=1000)
    plt.close()

    output_simulation_to_csv(
        times=t,
        positions=x,
        velocities=v,
        setpoints=np.array([simulation_params.setpoint]*len(t)),
        filename="msd_simulation_pi_control_equal_system_and_controller_freq.csv"
    )

def test_pi_controller_greater_system_frequency():

    # Frequencies
    dt_system = 0.0001  # System frequency (10000 Hz)
    dt_controller = 0.001  # Controller frequency (1000 Hz)

    # Create instances of the system and the controller
    msd_system = MassSpringDamper(m=msd_params.m, k=msd_params.k, b=msd_params.b)
    pi_controller = PIController(kp=pi_controller_params.kp, ki=pi_controller_params.ki, dt=dt_controller)
    simulation = Simulation(msd_system, pi_controller)

    # Perform the simulation
    t, x, v = simulation.simulate(
        simulation_params.setpoint,
        simulation_params.initial_conditions,
        simulation_params.duration,
        dt_system,
        dt_controller,
    )

    # Plot the results
    plt.plot(t, x, label='Position')
    plt.plot(t, [simulation_params.setpoint]*len(t), label='Setpoint', linestyle='--')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.title(f'Mass-Spring-Damper System Simulation with PI Controller\nSystem: m={msd_params.m}, k={msd_params.k}, b={msd_params.b}, dt={dt_system}\nController: Kp={pi_controller.kp}, Ki={pi_controller.ki}, dt={dt_controller}')
    plt.legend()
    plt.grid(True)
    plt.savefig('msd_simulation_pi_control_greater_system_freq.png', dpi=1000)
    plt.close()

    output_simulation_to_csv(
        times=t,
        positions=x,
        velocities=v,
        setpoints=np.array([simulation_params.setpoint]*len(t)),
        filename="msd_simulation_pi_control_greater_system_freq.csv"
    )

def test_pi_controller_greater_controller_frequency():

    # Frequencies
    dt_system = 0.001  # System frequency (1000 Hz)
    dt_controller = 0.0001  # Controller frequency (10000 Hz)

    # Create instances of the system and the controller
    msd_system = MassSpringDamper(m=msd_params.m, k=msd_params.k, b=msd_params.b)
    pi_controller = PIController(kp=pi_controller_params.kp, ki=pi_controller_params.ki, dt=dt_controller)
    simulation = Simulation(msd_system, pi_controller)

    # Perform the simulation
    t, x, v = simulation.simulate(
        simulation_params.setpoint,
        simulation_params.initial_conditions,
        simulation_params.duration,
        dt_system,
        dt_controller,
    )

    # Plot the results
    plt.plot(t, x, label='Position')
    plt.plot(t, [simulation_params.setpoint]*len(t), label='Setpoint', linestyle='--')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.title(f'Mass-Spring-Damper System Simulation with PI Controller\nSystem: m={msd_params.m}, k={msd_params.k}, b={msd_params.b}, dt={dt_system}\nController: Kp={pi_controller.kp}, Ki={pi_controller.ki}, dt={dt_controller}')
    plt.legend()
    plt.grid(True)
    plt.savefig('msd_simulation_pi_control_greater_controller_freq.png', dpi=1000)
    plt.close()

    output_simulation_to_csv(
        times=t,
        positions=x,
        velocities=v,
        setpoints=np.array([simulation_params.setpoint]*len(t)),
        filename="msd_simulation_pi_control_greater_controller_freq.csv"
    )
