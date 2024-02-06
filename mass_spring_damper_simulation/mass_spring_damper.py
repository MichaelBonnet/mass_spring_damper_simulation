"""
This module defines a MassSpringDamper class which models a basic mass-spring-damper system. 
Such a system is commonly found in mechanical engineering and physics to describe the motion 
of a mass connected to a spring and a damper. It's used to study the dynamics of systems 
under applied forces, taking into account mass, spring stiffness, and damping effects.
"""

# Standard library imports

# Third party imports

# Local imports


class MassSpringDamper:
    """A class to model a mass-spring-damper system.
    
    Attributes:
        m (float): Mass of the system in kilograms.
        k (float): Spring constant in Newtons per meter.
        b (float): Damping coefficient in Newton-seconds per meter.
    """
    
    def __init__(self, m: float, k: float, b: float):
        """Initializes the MassSpringDamper system with mass, spring constant, and damping coefficient.
        
        Args:
            m (float): Mass of the system in kilograms.
            k (float): Spring constant in Newtons per meter.
            b (float): Damping coefficient in Newton-seconds per meter.
        """
        self.m = m
        self.k = k
        self.b = b

    def compute_acceleration(self, x: float, v: float, force: float) -> float:
        """Computes the acceleration of the mass in the mass-spring-damper system under an applied force.
        
        The acceleration is calculated based on Newton's second law and takes into account the
        damping force and the restoring force from the spring.
        
        Args:
            x (float): The displacement of the mass from its equilibrium position in meters.
            v (float): The velocity of the mass in meters per second.
            force (float): The external force applied to the mass in Newtons.
        
        Returns:
            float: The acceleration of the mass in meters per second squared.
        """
        a = (force - self.b * v - self.k * x) / self.m
        return a
