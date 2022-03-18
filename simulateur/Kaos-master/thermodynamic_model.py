import numpy as np

def Pression(z, T):
    """
    Paramètres : 
    - z l'altitude (en m)
    - T la température ambiante (en K)
    Le calcul est basé sur le modèle d'une athmosphere (cf documentation)
    
    Retourne : la pression ambiante en Pa
    """
    P0 = 964.1 * pow(10, 2)  # Pa
    if z<11000:
        return np.round(P0 * pow(T/288.08, 5.256), 5)
    elif z<25000:
        return np.round(22650*np.exp(1.73-0.000157*z), 5)
    else:
        return np.round(2.488 * pow(T/216.6, -11.388), 5)

def Temperature(z, T0=None):
    """
    Paramètres : 
    - z : l'altitude (en m)
    - T0 : la température au sol (en K) (si non renseigner : T0 = 295.6 K soit 27.8°C)
    Le calcul est basé sur le modèle d'une athmosphere (cf documentation)
    
    Retourne : la température ambiante en K
    """
    if (T0 is None):
        T0 = 288+7.9

    if z<11000:
        return T0 - 0.00649 * z
    elif z<25000: 
        return 216.64
    else:
        return 141.89


def rho_air(z):
    """
    Paramètre :
    - z : l'altitude (en m)
    Modèle du gaz parfait
    
    Retourne : la densité de l'air
    """
    T = Temperature(z)
    P = Pression(z,T)
    M = 28.965 * pow(10, -3)  # kg.mol-1 à 1 bar
    R = 8.314  # constante des gaz parfait
    return P * M / (R * T)
