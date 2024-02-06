# Standard library imports

# Third party imports
import matplotlib.pyplot as plt
import numpy as np

# Local imports
from mass_spring_damper_simulation.mass_spring_damper import MassSpringDamper
from mass_spring_damper_simulation.simulation import Simulation
from mass_spring_damper_simulation.controller import PController, PIController, PDController, PIDController
from tests.utils import pid_controller_params, msd_params, simulation_params, output_simulation_to_csv


def test_just_pid_controller():

    # Frequencies
    dt_system = 0.5  # System frequency (10 Hz)
    dt_controller = 0.5  # Controller frequency (10 Hz)

    # Create instances of the system and the controller
    msd_system = MassSpringDamper(m=100.0, k=1.0, b=2.0)
    pid_controller = PIDController(kp=10.0, ki=0.05, kd=10.0, dt=dt_controller)
    pid_controller_simulation = Simulation(msd_system, pid_controller)

    # Perform the simulation
    t_pid, x_pid, v_pid = pid_controller_simulation.simulate(
        setpoint=0.5,
        initial_conditions=(1.0, 0.0),
        duration=300.0,
        dt_system=dt_system,
        dt_controller=dt_controller,
    )

    # Plot the results
    plt.plot(t_pid, x_pid, label='PID Position')
    plt.plot(t_pid, [0.5]*len(t_pid), label='Setpoint', linestyle='--')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.title(f'Mass-Spring-Damper System Simulation: PID Controller')
    plt.legend()
    plt.grid(True)
    plt.savefig('msd_simulation_only_pid_control.png', dpi=1000)
    plt.close()


def test_just_pid_controller_2():

    # Frequencies
    dt_system = 0.01  # System frequency (100 Hz)
    dt_controller = 0.01  # Controller frequency (100 Hz)

    # Create instances of the system and the controller
    msd_system = MassSpringDamper(m=1.0, k=2.0, b=1.0)
    pid_controller = PIDController(kp=10.0, ki=5.0, kd=2.0, dt=dt_controller)
    pid_controller_simulation = Simulation(msd_system, pid_controller)

    # Perform the simulation
    t_pid, x_pid, v_pid = pid_controller_simulation.simulate(
        setpoint=1.0,
        initial_conditions=(0.0, 0.0),
        duration=6.0,
        dt_system=dt_system,
        dt_controller=dt_controller,
    )

    # Plot the results
    plt.plot(t_pid, x_pid, label='PID Position')
    plt.plot(t_pid, [1.0]*len(t_pid), label='Setpoint', linestyle='--')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.title(f'Mass-Spring-Damper System Simulation: PID Controller')
    plt.legend()
    plt.grid(True)
    plt.savefig('msd_simulation_only_pid_control_2.png', dpi=1000)
    plt.close()



def test_just_p_controller_1():

    # Frequencies
    dt_system = 0.01  # System frequency (100 Hz)
    dt_controller = 0.01  # Controller frequency (100 Hz)

    # Create instances of the system and the controller
    msd_system = MassSpringDamper(m=1.0, k=20.0, b=10.0)
    p_controller = PController(kp=300.0)
    p_controller_simulation = Simulation(msd_system, p_controller)

    # Perform the simulation
    t_pid, x_pid, v_pid = p_controller_simulation.simulate(
        setpoint=1.0,
        initial_conditions=(0.0, 0.0),
        duration=1.2,
        dt_system=dt_system,
        dt_controller=dt_controller,
    )

    # Plot the results
    plt.plot(t_pid, x_pid, label='P Position')
    plt.plot(t_pid, [1.0]*len(t_pid), label='Setpoint', linestyle='--')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.title(f'Mass-Spring-Damper System Simulation: P Controller')
    plt.legend()
    plt.grid(True)
    plt.savefig('msd_simulation_only_p_control_1.png', dpi=1000)
    plt.close()


def test_just_pd_controller_1():

    # Frequencies
    dt_system = 0.01  # System frequency (100 Hz)
    dt_controller = 0.01  # Controller frequency (100 Hz)

    # Create instances of the system and the controller
    msd_system = MassSpringDamper(m=1.0, k=20.0, b=10.0)
    pd_controller = PDController(kp=300.0, kd=20.0, dt=dt_controller)
    pd_controller_simulation = Simulation(msd_system, pd_controller)

    # Perform the simulation
    t_pid, x_pid, v_pid = pd_controller_simulation.simulate(
        setpoint=1.0,
        initial_conditions=(0.0, 0.0),
        duration=1.2,
        dt_system=dt_system,
        dt_controller=dt_controller,
    )

    # Plot the results
    plt.plot(t_pid, x_pid, label='PD Position')
    plt.plot(t_pid, [1.0]*len(t_pid), label='Setpoint', linestyle='--')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.title(f'Mass-Spring-Damper System Simulation: PD Controller')
    plt.legend()
    plt.grid(True)
    plt.savefig('msd_simulation_only_pd_control_1.png', dpi=1000)
    plt.close()


def test_just_pi_controller_1():

    # Frequencies
    dt_system = 0.01  # System frequency (100 Hz)
    dt_controller = 0.01  # Controller frequency (100 Hz)

    # Create instances of the system and the controller
    msd_system = MassSpringDamper(m=1.0, k=20.0, b=10.0)
    pi_controller = PIController(kp=300.0, ki=1000.0, dt=dt_controller)
    pi_controller_simulation = Simulation(msd_system, pi_controller)

    # Perform the simulation
    t_pid, x_pid, v_pid = pi_controller_simulation.simulate(
        setpoint=1.0,
        initial_conditions=(0.0, 0.0),
        duration=1.8,
        dt_system=dt_system,
        dt_controller=dt_controller,
    )

    # Plot the results
    plt.plot(t_pid, x_pid, label='PI Position')
    plt.plot(t_pid, [1.0]*len(t_pid), label='Setpoint', linestyle='--')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.title(f'Mass-Spring-Damper System Simulation: PD Controller')
    plt.legend()
    plt.grid(True)
    plt.savefig('msd_simulation_only_pi_control_1.png', dpi=1000)
    plt.close()


def test_just_pid_controller_1():

    # Frequencies
    dt_system = 0.001  # System frequency (100 Hz)
    dt_controller = 0.001  # Controller frequency (100 Hz)

    # Create instances of the system and the controller
    msd_system = MassSpringDamper(m=1.0, k=20.0, b=10.0)
    pid_controller = PIDController(kp=300.0, ki=1000.0, kd=50.0, dt=dt_controller)
    pid_controller_simulation = Simulation(msd_system, pid_controller)

    # Perform the simulation
    t_pid, x_pid, v_pid = pid_controller_simulation.simulate(
        setpoint=1.0,
        initial_conditions=(0.0, 0.0),
        duration=1.8,
        dt_system=dt_system,
        dt_controller=dt_controller,
    )

    # Plot the results
    plt.plot(t_pid, x_pid, label='PID Position')
    plt.plot(t_pid, [1.0]*len(t_pid), label='Setpoint', linestyle='--')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.title(f'Mass-Spring-Damper System Simulation: PD Controller')
    plt.legend()
    plt.grid(True)
    plt.savefig('msd_simulation_only_pid_control_1.png', dpi=1000)
    plt.close()



def test_p_vs_pd_controller():

    # Frequencies
    dt_system = 0.1  # System frequency (100 Hz)
    dt_controller = 0.1  # Controller frequency (100 Hz)

    # Create instances of the system and the controller
    msd_system = MassSpringDamper(m=1.0, k=9.86960, b=6)
    p_controller = PController(kp=50.0)
    p_controller_simulation = Simulation(msd_system, p_controller)

    pd_controller = PDController(kp=50.0, kd=2.5, dt=dt_controller)
    pd_controller_simulation = Simulation(msd_system, pd_controller)

    # Perform the simulation
    t_p, x_p, v_p = p_controller_simulation.simulate(
        setpoint=1.0,
        initial_conditions=(0, 0),
        duration=3.0,
        dt_system=dt_system,
        dt_controller=dt_controller,
    )

    # Perform the simulation
    t_pd, x_pd, v_pd = pd_controller_simulation.simulate(
        setpoint=1.0,
        initial_conditions=(0, 0),
        duration=3.0,
        dt_system=dt_system,
        dt_controller=dt_controller,
    )

    # Plot the results
    plt.plot(t_p, x_p, label='P Position')
    plt.plot(t_p, x_pd, label='PD Position')
    plt.plot(t_p, [simulation_params.setpoint]*len(t_p), label='Setpoint', linestyle='--')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.title(f'Mass-Spring-Damper System Simulation: P vs PD Controller')
    plt.legend()
    plt.grid(True)
    plt.savefig('msd_simulation_p_vs_pd_control.png', dpi=1000)
    plt.close()


def test_p_vs_pi_controller():
    # Frequencies
    dt_system = 0.1  # System frequency (100 Hz)
    dt_controller = 0.1  # Controller frequency (100 Hz)

    # Create instances of the system and the controller
    msd_system = MassSpringDamper(m=1.0, k=9.86960, b=6)
    p_controller = PController(kp=50.0)
    p_controller_simulation = Simulation(msd_system, p_controller)

    pi_controller = PIController(kp=50.0, ki=50.0, dt=dt_controller)
    pi_controller_simulation = Simulation(msd_system, pi_controller)

    # Perform the simulation
    t_p, x_p, v_p = p_controller_simulation.simulate(
        setpoint=1.0,
        initial_conditions=(0, 0),
        duration=3.0,
        dt_system=dt_system,
        dt_controller=dt_controller,
    )

    # Perform the simulation
    t_pi, x_pi, v_pi = pi_controller_simulation.simulate(
        setpoint=1.0,
        initial_conditions=(0, 0),
        duration=3.0,
        dt_system=dt_system,
        dt_controller=dt_controller,
    )

    # Plot the results
    plt.plot(t_p, x_p, label='P Position')
    plt.plot(t_p, x_pi, label='PI Position')
    plt.plot(t_p, [simulation_params.setpoint]*len(t_p), label='Setpoint', linestyle='--')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.title(f'Mass-Spring-Damper System Simulation: P vs PD Controller')
    plt.legend()
    plt.grid(True)
    plt.savefig('msd_simulation_p_vs_pi_control.png', dpi=1000)
    plt.close()
