import os,sys,inspect
from typing import Iterable, Optional
current_dir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir) 

from rockets_builder import *
from engine_parser import *
from rockets.rocketclass import Rocket
import matplotlib.pyplot as plt

#########################################
########## Rocket definitions ###########
#########################################

rocket_name = Rocket(
        aux_masse = 0.0,
        moteur = Moteur(),

        parachutes = [Parachute()],
        corps = [Corps()],
        aileron =[Aileron()],
        ogive = Ogive(),
        jupe_retreints_list = [Jupe_retreint()],

        rcs = [RCS()],

        cn = None,
        cg = None,
        cpa = None,
        accelerometres_list=[Accelerometre()],
        gyroscopes_list = [Gyroscope()],
        gyrometres_list = [Gyrometre()],
        barometres_list = [Barometre()],
        magnetometres_list = [Magnetometre()],
        thermometres_list = [Thermometre()],
        GPS_list = [GPS()],
        
        list_Cx = np.array(
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
        ),
)


def affichage_fusée(rocket):
    plt.figure()
    my_rocket = rocket

    H=0
    L=0
    for i in my_rocket.corps:
        H+=i.H
        if i.d_corps>L:
            L=i.d_corps
    L+=2*max(i.e for i in my_rocket.aileron)

    for i in my_rocket.jupe_retreints_list:
        H+=i.H
    H+=my_rocket.ogive.H


    plt.axes()


    for i in my_rocket.corps:
        rectangle = plt.Rectangle((L/2-i.d_corps/2,H-i.X-i.H), i.d_corps, i.H, fill=False)
        plt.gca().add_patch(rectangle)
        
        
    for i in my_rocket.aileron:
        x = [L/2+i.d_Aileron/2,L/2+i.d_Aileron/2,L/2+i.d_Aileron/2+i.e,L/2+i.d_Aileron/2+i.e]
        y = [H-i.X_Aileron,H-i.X_Aileron-i.m,H-i.X_Aileron-i.n-i.p,H-i.X_Aileron-i.p]
        plt.gca().add_patch(plt.Polygon(xy=list(zip(x,y)), fill=False))
        
        x = [L/2-i.d_Aileron/2,L/2-i.d_Aileron/2,L/2-i.d_Aileron/2-i.e,L/2-i.d_Aileron/2-i.e]
        y = [H-i.X_Aileron,H-i.X_Aileron-i.m,H-i.X_Aileron-i.n-i.p,H-i.X_Aileron-i.p]
        plt.gca().add_patch(plt.Polygon(xy=list(zip(x,y)), fill=False))  
        
        
        
    for i in my_rocket.jupe_retreints_list:
        x = [L/2-i.d1/2,L/2+i.d1/2,L/2+i.d2/2,L/2-i.d2/2]
        y = [H-i.X,H-i.X,H-i.X-i.H,H-i.X-i.H]
        plt.gca().add_patch(plt.Polygon(xy=list(zip(x,y)), fill=False))
        
        

    x = [L/2,L/2+my_rocket.ogive.d_ogive/2,L/2-my_rocket.ogive.d_ogive/2]
    y = [H,H-my_rocket.ogive.H,H-my_rocket.ogive.H]
    plt.gca().add_patch(plt.Polygon(xy=list(zip(x,y)), fill=False))


    plt.axis('scaled')
    #plt.show()
