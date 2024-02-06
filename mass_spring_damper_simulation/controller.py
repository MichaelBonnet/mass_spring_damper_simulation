"""
This module defines various types of controllers used in control systems: Proportional (P), Proportional-Integral (PI),
Proportional-Derivative (PD), and Proportional-Integral-Derivative (PID) controllers. Each controller is designed to 
adjust an output based on the difference between a setpoint and a current process value, with the aim of reducing 
the error and stabilizing the system.

Classes:
    BaseController: An abstract base class for all controllers.
    PController: Implements a Proportional control logic.
    PIController: Implements Proportional-Integral control logic.
    PDController: Implements Proportional-Derivative control logic.
    PIDController: Implements Proportional-Integral-Derivative control logic.
"""

# Standard library imports
from abc import ABC, abstractmethod

# Third party imports

# Local imports


class BaseController(ABC):
    """
    An abstract base class that defines the common interface and initialization for all controllers. It requires 
    subclasses to implement the compute_control method, which calculates the control output based on the error 
    between the setpoint and the current measurement.

    Attributes:
        kp (float): Proportional gain.
        ki (float): Integral gain.
        kd (float): Derivative gain.
        dt (float): Time step delta.
    """

    @abstractmethod
    def __init__(self, kp: float, ki: float = 0.0, kd: float = 0.0, dt: float = 0.0) -> None:
        """
        Initialize the controller with gains and optionally time delta for those that need it.

        Args:
            kp (float): Proportional gain.
            ki (float): Integral gain. Default is 0.0 for controllers that don't use it.
            kd (float): Derivative gain. Default is 0.0 for controllers that don't use it.
            dt (float): Time step delta. Default is 0.0 for controllers that don't use it.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt

    @abstractmethod
    def compute_control(self, setpoint: float, measurement: float) -> float:
        """
        Compute the control output. This method needs to be implemented by each subclass.

        Args:
            setpoint (float): Desired setpoint value.
            measurement (float): Current measurement value.

        Returns:
            float: Control output.
        """
        pass


class PController(BaseController):
    """
    Implements a Proportional (P) controller that calculates the control output as the product of the proportional 
    gain (kp) and the error between the setpoint and the measurement. This controller aims to reduce the error by 
    adjusting the output proportionally.

    Attributes:
        kp (float): Proportional gain.
    """

    def __init__(self, kp: float) -> None:
        """
        Initialize the proportional controller with a proportional gain.

        Args:
            kp (float): Proportional gain.
        """
        super().__init__(kp=kp)
    
    def compute_control(self, setpoint: float, measurement: float) -> float:
        """
        Compute the control output using proportional logic.

        Args:
            setpoint (float): Desired setpoint value.
            measurement (float): Current measurement value.

        Returns:
            float: Control output.
        """
        error = setpoint - measurement
        return self.kp * error


class PIController(BaseController):
    """
    Implements a Proportional-Integral (PI) controller that combines proportional control with integral control, 
    which accounts for past values of the error by integrating them over time. This helps eliminate steady-state 
    errors and improves system stability.

    Attributes:
        kp (float): Proportional gain.
        ki (float): Integral gain.
        dt (float): Time step delta.
    """

    def __init__(self, kp: float, ki: float, dt: float) -> None:
        """
        Initialize the proportional-integral controller with gains and time delta.

        Args:
            kp (float): Proportional gain.
            ki (float): Integral gain.
            dt (float): Time step delta.
        """
        super().__init__(kp=kp, ki=ki, dt=dt)
        self.integral = 0
    
    def compute_control(self, setpoint: float, measurement: float) -> float:
        """
        Compute the control output using proportional and integral logic.

        Args:
            setpoint (float): Desired setpoint value.
            measurement (float): Current measurement value.

        Returns:
            float: Control output.
        """
        error = setpoint - measurement
        self.integral += error * self.dt
        return self.kp * error + self.ki * self.integral


class PDController(BaseController):
    """
    Implements a Proportional-Derivative (PD) controller that combines proportional control with derivative control, 
    which predicts system behavior by considering the rate of change of the error. This helps improve system 
    responsiveness and stability, especially in systems with high inertia.

    Attributes:
        kp (float): Proportional gain.
        kd (float): Derivative gain.
        dt (float): Time step delta.
    """

    def __init__(self, kp: float, kd: float, dt: float) -> None:
        """
        Initialize the proportional-derivative controller with gains and time delta.

        Args:
            kp (float): Proportional gain.
            kd (float): Derivative gain.
            dt (float): Time step delta.
        """
        super().__init__(kp=kp, kd=kd, dt=dt)
        self.previous_error = 0
    
    def compute_control(self, setpoint: float, measurement: float) -> float:
        """
        Compute the control output using proportional and derivative logic.

        Args:
            setpoint (float): Desired setpoint value.
            measurement (float): Current measurement value.

        Returns:
            float: Control output.
        """
        error = setpoint - measurement
        derivative = (error - self.previous_error) / self.dt
        self.previous_error = error
        return self.kp * error + self.kd * derivative


class PIDController(BaseController):
    """
    Implements a Proportional-Integral-Derivative (PID) controller, combining the features of P, PI, and PD 
    controllers. It calculates the control output by considering the current error, the integral of past errors, 
    and the derivative of the current error. This comprehensive approach allows for precise control, reducing 
    overshoot, and improving system stability.

    Attributes:
        kp (float): Proportional gain.
        ki (float): Integral gain.
        kd (float): Derivative gain.
        dt (float): Time step delta.
    """

    def __init__(self, kp: float, ki: float, kd: float, dt: float) -> None:
        """
        Initialize the proportional-integral-derivative controller with gains and time delta.

        Args:
            kp (float): Proportional gain.
            ki (float): Integral gain.
            kd (float): Derivative gain.
            dt (float): Time step delta.
        """
        super().__init__(kp=kp, ki=ki, kd=kd, dt=dt)
        self.previous_error = 0
        self.integral = 0
    
    def compute_control(self, setpoint: float, measurement: float) -> float:
        """
        Compute the control output using proportional, integral, and derivative logic.

        Args:
            setpoint (float): Desired setpoint value.
            measurement (float): Current measurement value.

        Returns:
            float: Control output.
        """
        error = setpoint - measurement
        self.integral += error * self.dt
        derivative = (error - self.previous_error) / self.dt
        self.previous_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative
