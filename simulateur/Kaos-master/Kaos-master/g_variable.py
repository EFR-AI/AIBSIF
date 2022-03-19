import numpy as np


def g_acceleration_pesanteur(latitude, altitude):
    """
    Latitude en degrés, altitude en mètres
    """
    phi = latitude/360*2*np.pi
    return 9.80616 - 0.025928*np.cos(2*phi) + 6.9*pow(10,-5)*np.cos(2*phi )**2 -3.086*pow(10,-6)*altitude

