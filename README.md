# Mass-Spring-Damper System Control Simulation
This repo is my work in response to the following prompt:

Your task is to build a closed-loop simulation of the following Mass-Spring-Damper system and implement a controller to control its position.

A few things to keep in mind...
* All parameters in the simulation should be configurable
* Keep the user in mind as you design the interface to the simulation
* One input to the simulation should be a desired position for the mass, and the controller should be able to bring the mass to that position
* The controller and the dynamics should be able to operate at separate frequencies
* The simulation should be scalable, so pay special attention to how it is structured, and how data is managed

This prompt is left purposefully open-ended, so feel free to get creative with your implementation and with any additional features that you think demonstrate your abilities.

When adding features, keep in mind that the goal of simulation is to validate the control performance, stability, and robustness of the algorithm, and to validate future designs.

## Usage

To use manually, do the following in a terminal:

1. `python3 -m venv myvenv`
2. On Linux/MacOS, `source myvenv/bin/activate`. On Windows (powershell), `venv\Scripts\Activate.ps1`
3. `python3 -m pip install -r requirements.txt`
4. `python3 main.py`

To run tests, do the following in a terminal:

1. `python3 -m venv myvenv`
2. On Linux/MacOS, `source myvenv/bin/activate`. On Windows (powershell), `venv\Scripts\Activate.ps1`
3. `python3 -m pip install -r requirements.txt`
4. `python3 -m pytest tests/`