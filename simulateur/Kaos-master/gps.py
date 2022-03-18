import numpy as np


class Coordonne_gps:
    def __init__(self, X0):
        self.X0 = X0

    def position_GPS(self, X):
        """
        Paramètres :X0 = [lat,lon] en degrés décimaux. La position initiale de la fusée
        X : Position de la fusée dans le repère terrestre (x pointe vers le nord)
        Retourne la position GPS de la fusée en degrés décimaux
        """
        # Calcul des paramètres géométrique de la Terre au niveau de la fusée
        R = 6371000
        PT = 2 * np.pi * R
        h = R * np.sin(self.X0[0] / 180 * np.pi)
        r = np.sqrt(R ** 2 + h ** 2)
         # Circonférence de la Terre au niveau du plan de coupe dans lequel se trouve la fusée
        P = (2 * np.pi * r) 

        # Calcul de la variation de coordonnées
        dx = X[1] * 360 / P
        dy = X[0] * 360 / PT
        return np.array([self.X0[0] + dy, self.X0[1] + dx, -X[2]])


def convert_dd_dms(X):
    """
    Paramètre : X = [lat,lon], une coordonnée GPS en degrés décimaux
    Retourne cette position GPS en DMS    (Degré Minute Seconde)

    """
    # Conversion des valeures dd en DMS
    lat_d = abs(int(X[0]))
    lon_d = abs(int(X[1]))
    lat_min = int((X[0] % 1) * 60)
    lon_min = int((X[1] % 1) * 60)
    lat_sec = round(((X[0] % 1) * 60) % 1 * 60, 3)
    lon_sec = round(((X[1] % 1) * 60) % 1 * 60, 3)

    # Détermination du demi hémisphère.

    if X[0] < 0:
        L = "s"
        lat_min = 59 - lat_min
        lat_sec = 60 - lat_sec
    else:
        L = "n"
    if X[1] < 0:
        l = "o"
        lon_min = 59 - lon_min
        lon_sec = 60 - lon_sec
    else:
        l = "e"
    return np.array([[L, l], [lat_d, lat_min, lat_sec, lon_d, lon_min, lon_sec]])


def convert_dms_dd(X):
    """
    Paramètre : X = ([[L,l],[lat_d,lat_min,lat_sec,lon_d,lon_min,lon_sec]]) : Position GPS en DMS
    Retourne cette position GPS en DD

    """
    if X[0][0] == "n":
        lat = round(X[1][0] + X[1][1] / 60 + X[1][2] / 3600, 5)
    else:
        lat = round(-X[1][0] - X[1][1] / 60 - X[1][2] / 3600, 5)
    if X[0][1] == "e":
        lon = round(X[1][3] + X[1][4] / 60 + X[1][5] / 3600, 5)
    else:
        lon = round(-X[1][3] - X[1][4] / 60 - X[1][5] / 3600, 5)
    return np.array([lat, lon])
