# Third party imports
import matplotlib.pyplot as plt

# Local imports
from mass_spring_damper_simulation.mass_spring_damper import MassSpringDamper
from mass_spring_damper_simulation.simulation import Simulation
from mass_spring_damper_simulation.controller import PController, PIController, PDController, PIDController
from tests.utils import output_simulation_to_csv

# Prompting for user input

print("Hello! Welcome to the Mass-Spring-Damper System Simulation Tool.")
print("You will be prompted for system and controller parameters.")
mass = float(input("Enter desired mass (kg): "))
spring_constant = float(input("Enter desired spring constant or k (Nm): "))
damping_force = float(input("Enter desired damping force or b (Nsm): "))
msd_system = MassSpringDamper(m=mass, k=spring_constant, b=damping_force)
print(f"You have input the following mass-spring-damper system parameters:")
print(f"mass (m)            = {mass} kilograms")
print(f"spring constant (k) = {spring_constant} Newton-meters")
print(f"damping force (b)   = {damping_force} Newton-second-meters")
input_controller = input("Enter desired controller (enter P, PI, PD, or PID): ")
controller_type = ""
match input_controller:
    case "P":
        controller_type = "P"
        kp = float(input("Enter desired P Controller Kp gain: "))
        controller = PController(kp=kp)
        print(f"You have input the following controller parameters: P Controller, Kp={kp}")
    case "PI":
        controller_type = "PI"
        kp = float(input("Enter desired PI Controller Kp gain: "))
        ki = float(input("Enter desired PI Controller Ki gain: "))
        dt_controller = float(input("Enter desired controller time step in seconds (ex: 0.001): "))
        controller = PIController(kp=kp, ki=ki, dt=dt_controller)
        print(f"You have input the following controller parameters: PI Controller, Kp={kp}, Ki={ki}")
    case "PD":
        controller_type = "PD"
        kp = float(input("Enter desired PD Controller Kp gain: "))
        kd = float(input("Enter desired PD Controller Kd gain: "))
        dt_controller = float(input("Enter desired controller time step in seconds (ex: 0.001): "))
        controller = PDController(kp=kp, kd=kd, dt=dt_controller)
        print(f"You have input the following controller parameters: PD Controller, Kp={kp}, Kd={kd}")
    case "PID":
        controller_type = "PID"
        kp = float(input("Enter desired PID Controller Kp gain: "))
        ki = float(input("Enter desired PID Controller Ki gain: "))
        kd = float(input("Enter desired PID Controller Kd gain: "))
        dt_controller = float(input("Enter desired controller time step in seconds (ex: 0.001): "))
        controller = PIDController(kp=kp, ki=ki, kd=kd, dt=dt_controller)
        print(f"You have input the following controller parameters: PID Controller, Kp={kp}, Ki={ki} Kd={kd}")
    case _:
        print(f"Improper value {input_controller} entered (must be one of P, PI, PD, or PID). Program exiting.")
        exit()

setpoint = float(input("Enter desired final system position in meters (ex: 1): "))
initial_position = float(input("Enter desired initial system position in meters (ex: 0): "))
initial_velocity = float(input("Enter desired initial system position in meters/second (ex: 0): "))
duration = float(input("Enter desired simulation duration in seconds (ex: 2): "))
dt_system = float(input("Enter desired system time step in seconds (ex: 0.001): "))
simulation = Simulation(
    system=msd_system,
    controller=controller
)
if controller_type == "P":
    dt_controller = dt_system
print(f"You have input the following simulation parameters:")
print(f"setpoint               = {setpoint} meters")
print(f"initial position       = {initial_position} meters")
print(f"initial velocity       = {initial_velocity} meters per second")
print(f"duration               = {duration} seconds")
print(f"system time step (dt)  = {dt_system} seconds")

save_data = input("A graph of the results will pop up. Do you want to save simulation time, position, and velocity data to a CSV? (Y/N): ")
t, x, v = simulation.simulate(
    setpoint=setpoint,
    initial_conditions=(initial_position, initial_velocity),
    duration=duration,
    dt_system=dt_system,
    dt_controller=dt_controller,
)

match save_data:
    case "Y":
        output_simulation_to_csv(times=t, positions=x, velocities=v, filename="simulation_data.csv")
        print("simulation data saved to simulation_data.csv")
    case "y":
        output_simulation_to_csv(times=t, positions=x, velocities=v, filename="simulation_data.csv")
        print("simulation data saved to simulation_data.csv")
    case "N":
        print("simulation data not saved")
    case "n":
        print("simulation data not saved")
    case _:
        print(f"Improper value {save_data} entered (must be one of Y, N, y, or n). Program exiting.")
        exit()

match controller_type:
    case "P":
        controller_line = f"Controller: Kp={kp}"
    case "PI":
        controller_line = f"Controller: Kp={kp}, Ki={ki}, dt={dt_controller}"
    case "PD":
        controller_line = f"Controller: Kp={kp}, Kd={kd}, dt={dt_controller}"
    case "PID":
        controller_line = f"Controller: Kp={kp}, Ki={ki}, Kd={kd}, dt={dt_controller}"

plt.plot(t, x, label='Position')
plt.plot(t, [setpoint]*len(t), label='Setpoint', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.title(f'Mass-Spring-Damper System Simulation with {controller_type} Controller\nSystem: m={mass}, k={spring_constant}, b={damping_force}, dt={dt_system}\n{controller_line}')
plt.legend()
plt.grid(True)
plt.savefig('msd_simulation.png', dpi=1000)
plt.show()