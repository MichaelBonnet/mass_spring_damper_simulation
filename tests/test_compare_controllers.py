# Standard library imports

# Third party imports
import matplotlib.pyplot as plt
import numpy as np

# Local imports
from mass_spring_damper_simulation.mass_spring_damper import MassSpringDamper
from mass_spring_damper_simulation.simulation import Simulation
from mass_spring_damper_simulation.controller import PController, PIController, PDController, PIDController
from tests.utils import ControllerParams, MassSpringDamperSystemParams

iis_controller_params = ControllerParams(kp=300.0, ki=1000.0, kd=50.0)
iis_msd_params = MassSpringDamperSystemParams(m=1.0, k=20.0, b=10.0)

def test_mooiman_pid_controller():

    # taken from:
    # https://content.oss.deltares.nl/delft3d/pid-controller.pdf

    # Frequencies
    dt_system = 0.5  # System frequency (10 Hz)
    dt_controller = 0.5  # Controller frequency (10 Hz)

    # Create instances of the system and the controller
    msd_system = MassSpringDamper(m=100.0, k=1.0, b=2.0)
    pid_controller = PIDController(kp=10.0, ki=0.05, kd=10.0, dt=dt_controller)
    pid_controller_simulation = Simulation(msd_system, pid_controller)

    # Perform the simulation
    t_pid, x_pid, _ = pid_controller_simulation.simulate(
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
    plt.savefig('msd_simulation_mooiman_pid_control.png', dpi=1000)
    plt.close()


def test_p_vs_pi_vs_pid_controller_iis():

    # taken from:
    # https://docslib.org/doc/877949/lecture-4-pid-of-a-spring-mass-damper-system

    # Frequencies
    dt_system = 0.001  # System frequency (1000 Hz)
    dt_controller = 0.001  # Controller frequency (1000 Hz)

    # Instantiate the system
    msd_system = MassSpringDamper(
        m=iis_msd_params.m,
        k=iis_msd_params.k,
        b=iis_msd_params.b
    )

    # Instantiate P controller + simulation
    p_controller = PController(kp=iis_controller_params.kp)
    p_controller_simulation = Simulation(msd_system, p_controller)

    # Instantiate PI controller + simulation
    pi_controller = PIController(
        kp=iis_controller_params.kp,
        ki=iis_controller_params.ki,
        dt=dt_controller
    )
    pi_controller_simulation = Simulation(msd_system, pi_controller)

    # Instantiate PID controller + simulation
    pid_controller = PIDController(
        kp=iis_controller_params.kp,
        ki=iis_controller_params.ki,
        kd=iis_controller_params.kd,
        dt=dt_controller
    )
    pid_controller_simulation = Simulation(msd_system, pid_controller)

    # Perform each simulation

    t_p, x_p, _ = p_controller_simulation.simulate(  # P Controller Simulation
        setpoint=1.0,
        initial_conditions=(0.0, 0.0),
        duration=1.8,
        dt_system=dt_system,
        dt_controller=dt_controller,
    )

    _, x_pi, _ = pi_controller_simulation.simulate(  # PI Controller Simulation
        setpoint=1.0,
        initial_conditions=(0.0, 0.0),
        duration=1.8,
        dt_system=dt_system,
        dt_controller=dt_controller,
    )

    _, x_pid, _ = pid_controller_simulation.simulate(  # PID Controller Simulation
        setpoint=1.0,
        initial_conditions=(0.0, 0.0),
        duration=1.8,
        dt_system=dt_system,
        dt_controller=dt_controller,
    )

    # Plot the results
    plt.plot(t_p, x_p, label='P Controller')
    plt.plot(t_p, x_pi, label='PI Controller')
    plt.plot(t_p, x_pid, label='PID Controller')
    plt.plot(t_p, [1.0]*len(t_p), label='Setpoint', linestyle='--')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.title(f'Mass-Spring-Damper System Simulation\nP vs PI vs PID Controller\nKp=300, Ki=1000, Kd=50')
    plt.legend()
    plt.grid(True)
    plt.savefig('msd_simulation_p_vs_pi_vs_pid_iis.png', dpi=1000)
    plt.close()


def test_p_vs_pd_controller_worcester():

    # taken from:
    # https://colin-price.wbs.uni.worc.ac.uk/Courses_2020_21/Comp3352/Worksheets_MiniLecture_Devel/PID/Chapter11_Some_PID_Theory.pdf

    # Frequencies
    dt_system = 0.1  # System frequency (100 Hz)
    dt_controller = 0.1  # Controller frequency (100 Hz)

    # Instantiate system and the controller
    msd_system = MassSpringDamper(m=1.0, k=9.86960, b=6)
    p_controller = PController(kp=50.0)
    p_controller_simulation = Simulation(msd_system, p_controller)

    pd_controller = PDController(kp=50.0, kd=2.5, dt=dt_controller)
    pd_controller_simulation = Simulation(msd_system, pd_controller)

    # Perform each simulation

    t_p, x_p, _ = p_controller_simulation.simulate(  # P Controller Simulation
        setpoint=1.0,
        initial_conditions=(0, 0),
        duration=3.0,
        dt_system=dt_system,
        dt_controller=dt_controller,
    )

    _, x_pd, _ = pd_controller_simulation.simulate(  # PD Controller Simulation
        setpoint=1.0,
        initial_conditions=(0, 0),
        duration=3.0,
        dt_system=dt_system,
        dt_controller=dt_controller,
    )

    # Plot the results
    plt.plot(t_p, x_p, label='P Controller')
    plt.plot(t_p, x_pd, label='PD Controller')
    plt.plot(t_p, [1.0]*len(t_p), label='Setpoint', linestyle='--')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.title(f'Mass-Spring-Damper System Simulation: P vs PD Controller')
    plt.legend()
    plt.grid(True)
    plt.savefig('msd_simulation_p_vs_pd_control_worcester.png', dpi=1000)
    plt.close()


def test_p_vs_pi_controller_worcester():

    # taken from:
    # https://colin-price.wbs.uni.worc.ac.uk/Courses_2020_21/Comp3352/Worksheets_MiniLecture_Devel/PID/Chapter11_Some_PID_Theory.pdf

    # Frequencies
    dt_system = 0.001  # System frequency (1000 Hz)
    dt_controller = 0.001  # Controller frequency (1000 Hz)

    # Instantiate the system
    msd_system = MassSpringDamper(m=1.0, k=9.86960, b=6)

    # Instantiate the P controller + simulation
    p_controller = PController(kp=50.0)
    p_controller_simulation = Simulation(msd_system, p_controller)

    # Instantiate the PI controller + simulation
    pi_controller = PIController(kp=25.0, ki=25.0, dt=dt_controller)
    pi_controller_simulation = Simulation(msd_system, pi_controller)

    # Perform each simulation

    t_p, x_p, _ = p_controller_simulation.simulate(  # P Controller Simulation
        setpoint=1.0,
        initial_conditions=(0, 0),
        duration=3.0,
        dt_system=dt_system,
        dt_controller=dt_controller,
    )

    _, x_pi, _ = pi_controller_simulation.simulate(  # PI Controller Simulation
        setpoint=1.0,
        initial_conditions=(0, 0),
        duration=3.0,
        dt_system=dt_system,
        dt_controller=dt_controller,
    )

    # Plot the results
    plt.plot(t_p, x_p, label='P Controller')
    plt.plot(t_p, x_pi, label='PI Controller')
    plt.plot(t_p, [1.0]*len(t_p), label='Setpoint', linestyle='--')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.title(f'Mass-Spring-Damper System Simulation: P vs PI Controller')
    plt.legend()
    plt.grid(True)
    plt.savefig('msd_simulation_p_vs_pi_control_worcester.png', dpi=1000)
    plt.close()


def test_pid_controller_worcester():

    # taken from:
    # https://colin-price.wbs.uni.worc.ac.uk/Courses_2020_21/Comp3352/Worksheets_MiniLecture_Devel/PID/Chapter11_Some_PID_Theory.pdf

    # Frequencies
    dt_system = 0.001  # System frequency (1000 Hz)
    dt_controller = 0.001  # Controller frequency (1000 Hz)

    # Instantiate the system and the controller
    msd_system = MassSpringDamper(m=1.0, k=9.86960, b=6)
    pid_controller = PIDController(kp=50.0, ki=45.0, kd=8.0, dt=dt_controller)
    pid_controller_simulation = Simulation(msd_system, pid_controller)

    # Perform the simulation
    t_pid, x_pid, _ = pid_controller_simulation.simulate(
        setpoint=1.0,
        initial_conditions=(0, 0),
        duration=3.0,
        dt_system=dt_system,
        dt_controller=dt_controller,
    )

    # Plot the results
    plt.plot(t_pid, x_pid, label='PID Controller')
    plt.plot(t_pid, [1.0]*len(t_pid), label='Setpoint', linestyle='--')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.title(f'Mass-Spring-Damper System Simulation: PID Controller')
    plt.legend()
    plt.grid(True)
    plt.savefig('msd_simulation_pid_control_worcester.png', dpi=1000)
    plt.close()
