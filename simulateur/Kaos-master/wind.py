from fonction_interpolation import *


# Use wind_parser.sh on wind profile from weather balloon to get alt, dir and mag files
# For now, the wind profile is fixed, it should become an argument later down the line


def list_maker(filepath):

    """
    This function is used to read a file in filepath and create a list from it (iterating on each line)
    """

    l = []
    with open(filepath) as f:
        l = [float(line.strip()) for line in f]

    return l


class ActiveWind:
    def __init__(
        self, wind_profile
    ):  # Se referer au document vol-de-la-fusee.pdf dans le drive /simulation/etat de l'art  à partir de la page 19
        self.alt = list_maker("wind/" + wind_profile + "/alt")
        self.alt.reverse()
        self.mag = list_maker("wind/" + wind_profile + "/mag")
        self.mag.reverse()
        self.hdg = list_maker("wind/" + wind_profile + "/dir")
        self.hdg.reverse()
        self.mag = np.column_stack((self.alt, self.mag))
        self.hdg = np.column_stack((self.alt, self.hdg))

    @property
    def departure_wind_heading(self):
        """
        Returns wind heading at ground level
        """

        return np.deg2rad(self.hdg[0, 1])

    def return_wind(self, h):
        """
        return wind vector at height h
        """
        return (
            np.array(
                [
                    interpolation(self.mag, h)
                    * np.cos(np.deg2rad(interpolation(self.hdg, h))),
                    interpolation(self.mag, h)
                    * np.sin(np.deg2rad(interpolation(self.hdg, h))),
                    0,
                ]
            )
            * -1
        )
