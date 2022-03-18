import os,sys,inspect
current_dir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir) 

from rockets_builder import *
from engine_parser import *

class MyRocket:
    def __init__(self):
        self.aux_masse = 
        self.nb_aileron = 
        self.moteur = Moteur(engine_reader('rockets/engines/thrust_profile.txt'), masse) 
        self.parachutes = [Parachute()] 
        self.corps = Corps()
        self.aileron = Aileron()
        self.ogive = Ogive() 
        self.rcs = [RCS()]
        jupe_retreints_list = [Jupe_retreint()]
         
        self.fusee = Fusee(aux_masse, self.corps, self.moteur, nb_aileron, self.aileron, self.ogive, jupe_retreints_list, self.parachutes, self.rcs) 
        self.inertie = inertie(self.fusee)
        self.list_Cx = np.array(
        [
        [0.1, 0.49458948],
        [0.7, 0.62356508],
        [0.8, 0.68864667],
        [0.85, 0.61258763],
        [0.9, 0.63427252],
        [0.95, 0.68763119],
        [1, 0.82986492],
        [1.05, 0.86513013],
        [1.1, 0.89168829],
        [1.2, 0.88589495],
        [2, 0.57830435],
        ]
        )