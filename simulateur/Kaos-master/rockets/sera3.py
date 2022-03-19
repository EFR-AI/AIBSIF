import os,sys,inspect
from random import random
import matplotlib.pyplot as plt
from typing import Iterable, Optional
current_dir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir) 

from rockets_builder import *
from engine_parser import *
from rockets.rocketclass import Rocket

#########################################
########## Rocket definitions ###########
#########################################

global sera3

sera3 = Rocket(
        aux_masse = 3.168,
        moteur = Moteur(thrust=engine_reader('rockets/engines/thrust_profile_SERA3.txt'), masse=14.294, h=0.01, R=0.125, X = 4.7),                       

        parachutes = [Parachute(surface=0.46,masse=0.088,X=1.85,alt=5100),Parachute(surface=4.64,masse=0.850,X=1.85,alt=500)],                 
        corps = [Corps(H=1.200,d_corps=0.250,masse=4.086,masse_ergol=26.041,X=3.500),Corps(H=2.800,d_corps=0.160,masse=22.137,masse_ergol=0,X=0.450)],           
        aileron =[ Aileron(e=0.300,n=0.090,p=0.260,m=0.350,d_Aileron=0.250,X_Aileron=4.350,masse=0.986,nb_ailerons=4),Aileron(e=0.180,n=0.190,p=0.160,m=0.350,d_Aileron=0.160,X_Aileron=2.9,masse=0.600,nb_ailerons=3)],  

        ogive = Ogive(H=0.450,d_ogive=0.160,masse = 3.665,forme="ogivale"),                                              
        jupe_retreints_list = [Jupe_retreint(H=0.250,X=3.25,d1=0.160,d2=0.250,masse=2.831)],                                       
                       

        rcs = None,

        cn = None,
        cg = None,
        cpa = None,
        accelerometres_list=[Accelerometre([2.09,0.0,0.0],phi=0.0, theta=0.0, psi=0.0, incertitude_phi=0.0, incertitude_theta=0.0, incertitude_psi=0.0, noise_power=random.randint(0,7)/10, erreur_de_justesse=random.randint(0,7)/10)],
        gyroscopes_list = [Gyroscope('zyx', noise_power=random.randint(0,7)/10, erreur_de_justesse=random.randint(0,7)/10)],
        gyrometres_list = [Gyrometre(noise_power=random.randint(0,7)/10, erreur_de_justesse=random.randint(0,7)/10)],
        barometres_list = None,
        magnetometres_list = [Magnetometre(noise_power=random.randint(0,7)/10, erreur_de_justesse=random.randint(0,7)/10)],
        thermometres_list = None,
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

print(sera3)
print(sera3.gyrometres_list[-1])


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

#affichage_fusée(SERA3)