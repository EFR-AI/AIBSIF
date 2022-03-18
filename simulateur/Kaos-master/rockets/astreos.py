import os,sys,inspect
from typing import Iterable, Optional
current_dir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir) 

from rockets_builder import *
from engine_parser import *
from rockets.rocketclass import Rocket

Astreos = Rocket(
    moteur = Moteur(engine_reader('rockets/engines/thrust_20S_astreos.txt'), 86,0.2,0.25,6),
    parachutes = [Parachute(12.57, 0.345, 0.2, None, True), Parachute(50.26, 3.905, 0.2, 200,False)],
    corps = [Corps(5.50,0.253, 11.603, 33,0.50)],
    aileron = [Aileron(0.30, 0.09, 0.21, 0.35, 0.253, 5.65, 0.599, 4)], #Se referer au modèle open rocket pour les dimensions
    ogive = Ogive(0.50, 0.253, 0.967), #Se referer au modèle open rocket pour les dimensions
    jupe_retreint_list = None,
    rcs = [RCS(0, 0.60, 100, 2), RCS(np.pi, 0.60, 100, 2), RCS(np.pi/2, 0.60, 100, 2), RCS(-np.pi/2, 0.60, 100, 2),
                            RCS(0, 5.90, 100, 2), RCS(np.pi, 5.90, 100, 2), RCS(np.pi/2, 5.90, 100, 2), RCS(-np.pi/2, 5.90, 100, 2)],
        
        # Masse qui ne proviennent pas explicitement d'une partie (masse inerte, anneau de serrage...)
    aux_masse = 3 + 12 * 0.5 + 5.964,

        #Cnalpha 
    cn = None,
        # Centre de gravité
    cg = 3.24,
        # Centre de portance 
    cpa = None,
        # Point de référence aéro
    # o = 3.30,
    # Coeff = [0,0,0,0],


    # fusee = Fusee(self.aux_masse, self.corps, self.moteur, self.aileron, 
    #                         self.ogive, self.jupe_retreint_list, self.parachutes, self.rcs, self.cn, self.cg, self.cpa,self.o,self.Coeff)
    #     self.inertie = inertie(self.fusee)
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
        accelerometres_list=[Accelerometre([2.09,0.0,0.0],phi=0.0, theta=0.0, psi=0.0, incertitude_phi=0.0, incertitude_theta=0.0, incertitude_psi=0.0, noise_power=0.0, erreur_de_justesse=0.0)],
        gyroscopes_list = [Gyroscope('zyx')],
        barometres_list = None,
        magnetometres_list = [Magnetometre()],
        thermometres_list = None,
        GPS_list = [GPS()])



# thrust_10S_astreos = [[0.100, 760.3],
#                     [0.200, 1200.0],
#                     [0.300, 2500.0],
#                     [0.400, 5000.0],
#                     [0.500, 5000.0],
#                     [0.600, 5000.0],
#                     [0.700, 5000.0],
#                     [0.800, 5000.0],
#                     [0.900, 5000.0],
#                     [1.000, 5000.0],
#                     [2.000, 5000.0],
#                     [3.000, 5000.0],
#                     [4.000, 5000.0],
#                     [5.000, 5000.0],
#                     [6.000, 5000.0],
#                     [7.000, 5000.0],
#                     [8.000, 5000.0],
#                     [9.000, 5000.0],
#                     [10.000, 5000.0],
#                     [10.100, 0.0]]


# thrust_15S_astreos = [[0.100,760.3],
#                      [0.200, 1200.0],
#                      [0.300, 2500.0],
#                      [0.400, 5000.0],
#                      [0.500, 5000.0],
#                      [0.600, 5000.0],
#                      [0.700, 5000.0],
#                      [0.800, 5000.0],
#                      [0.900, 5000.0],
#                      [1.000, 5000.0],
#                      [2.000, 5000.0],
#                      [3.000, 5000.0],
#                      [4.000, 5000.0],
#                      [5.000, 5000.0],
#                      [6.000, 5000.0],
#                      [7.000, 5000.0],
#                      [8.000, 5000.0],
#                      [9.000, 5000.0],
#                      [10.000, 5000.0],
#                      [11.000, 5000.0],
#                      [12.000, 5000.0],
#                      [13.000, 5000.0],
#                      [14.000, 5000.0],
#                      [15.000, 5000.0],
#                      [15.100, 0.0]]

import matplotlib.pyplot as plt
plt.figure()
my_rocket = Astreos

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
