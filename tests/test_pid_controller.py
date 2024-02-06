# Standard library imports

# Third party imports
import matplotlib.pyplot as plt
import numpy as np

# Local imports
from mass_spring_damper_simulation.mass_spring_damper import MassSpringDamper
from mass_spring_damper_simulation.simulation import Simulation
from mass_spring_damper_simulation.controller import PIDController
from tests.utils import pid_controller_params, msd_params, simulation_params, output_simulation_to_csv


def test_pid_controller_equal_frequencies():

    # Frequencies
    dt_system = 0.01  # System frequency (100 Hz)
    dt_controller = 0.01  # Controller frequency (100 Hz)

    # Create instances of the system and the controller
    msd_system = MassSpringDamper(m=msd_params.m, k=msd_params.k, b=msd_params.b)
    pid_controller = PIDController(kp=pid_controller_params.kp, ki=pid_controller_params.ki, kd=pid_controller_params.kd, dt=dt_controller)
    simulation = Simulation(msd_system, pid_controller)

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
    plt.title(f'Mass-Spring-Damper System Simulation with PID Controller\nSystem: m={msd_params.m}, k={msd_params.k}, b={msd_params.b}, dt={dt_system}\nController: Kp={pid_controller.kp}, Ki={pid_controller.ki}, Kd={pid_controller.kd}, dt={dt_controller}')
    plt.legend()
    plt.grid(True)
    plt.savefig('msd_simulation_pid_control_equal_system_and_controller_freq.png', dpi=1000)
    plt.close()

    output_simulation_to_csv(
        times=t,
        positions=x,
        velocities=v,
        setpoints=np.array([simulation_params.setpoint]*len(t)),
        filename="msd_simulation_pid_control_equal_system_and_controller_freq.csv"
    )

def test_pid_controller_greater_system_frequency():

    # Frequencies
    dt_system = 0.001  # System frequency (1000 Hz)
    dt_controller = 0.01  # Controller frequency (100 Hz)

    # Create instances of the system and the controller
    msd_system = MassSpringDamper(m=msd_params.m, k=msd_params.k, b=msd_params.b)
    pid_controller = PIDController(kp=pid_controller_params.kp, ki=pid_controller_params.ki, kd=pid_controller_params.kd, dt=dt_controller)
    simulation = Simulation(msd_system, pid_controller)

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
    plt.title(f'Mass-Spring-Damper System Simulation with PID Controller\nSystem: m={msd_params.m}, k={msd_params.k}, b={msd_params.b}, dt={dt_system}\nController: Kp={pid_controller.kp}, Ki={pid_controller.ki}, Kd={pid_controller.kd}, dt={dt_controller}')
    plt.legend()
    plt.grid(True)
    plt.savefig('msd_simulation_pid_control_greater_system_freq.png', dpi=1000)
    plt.close()

    output_simulation_to_csv(
        times=t,
        positions=x,
        velocities=v,
        setpoints=np.array([simulation_params.setpoint]*len(t)),
        filename="msd_simulation_pid_control_greater_system_freq.csv"
    )

def test_pid_controller_greater_controller_frequency():

    # Frequencies
    dt_system = 0.01  # System frequency (100 Hz)
    dt_controller = 0.001  # Controller frequency (1000 Hz)

    # Create instances of the system and the controller
    msd_system = MassSpringDamper(m=msd_params.m, k=msd_params.k, b=msd_params.b)
    pid_controller = PIDController(kp=pid_controller_params.kp, ki=pid_controller_params.ki, kd=pid_controller_params.kd, dt=dt_controller)
    simulation = Simulation(msd_system, pid_controller)

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
    plt.title(f'Mass-Spring-Damper System Simulation with PID Controller\nSystem: m={msd_params.m}, k={msd_params.k}, b={msd_params.b}, dt={dt_system}\nController: Kp={pid_controller.kp}, Ki={pid_controller.ki}, Kd={pid_controller.kd}, dt={dt_controller}')
    plt.legend()
    plt.grid(True)
    plt.savefig('msd_simulation_pid_control_greater_controller_freq.png', dpi=1000)
    plt.close()

    output_simulation_to_csv(
        times=t,
        positions=x,
        velocities=v,
        setpoints=np.array([simulation_params.setpoint]*len(t)),
        filename="msd_simulation_pid_control_greater_controller_freq.csv"
    )
