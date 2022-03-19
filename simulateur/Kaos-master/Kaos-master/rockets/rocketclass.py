
from typing import Iterable
from rockets_builder import Gyrometre, Moteur, Parachute,Corps,Aileron,Ogive,Jupe_retreint,RCS,Accelerometre,Gyroscope,Barometre,Thermometre,Magnetometre,GPS,Fusee,inertie
from dataclasses import dataclass

@dataclass
class Rocket:

    aux_masse: float  
    moteur : Moteur
    parachutes : 'list[Parachute]'    
    corps : 'list[Corps]'            
    aileron : 'list[Aileron]'  

    ogive : Ogive                                          
    jupe_retreints_list : 'list[Jupe_retreint]'                   

    rcs : 'list[RCS]'

    cn : float
    cg : float
    cpa : float
    accelerometres_list : 'list[Accelerometre]'
    gyroscopes_list : 'list[Gyroscope]'
    gyrometres_list : 'list[Gyrometre]'
    barometres_list : 'list[Barometre]'
    magnetometres_list : 'list[Magnetometre]'
    thermometres_list  : 'list[Thermometre]'
    GPS_list : 'list[GPS]'
    list_Cx : Iterable

    def __post_init__ (self):
        self.fusee = Fusee(self.aux_masse, self.corps, self.moteur, self.aileron,
                        self.ogive, self.jupe_retreints_list, self.parachutes, 
                        self.accelerometres_list, self.gyroscopes_list, self.gyrometres_list, self.barometres_list, self.magnetometres_list, self.thermometres_list, self.GPS_list, 
                        self.rcs, self.cn, self.cg, self.cpa) 
        self.inertie = inertie(self.fusee)