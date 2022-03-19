import os,sys,inspect
current_dir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir) 

from rockets_builder import *
from engine_parser import *

class MyRocket:
    def __init__(self):
        self.aux_masse = 800 # Input all and any masses in the rocket that is not a standard part
        self.nb_aileron = 3 # Number of ailerons
        self.moteur = Moteur(engine_reader('rockets/engines/thrust_profile.txt'), masse) 
        self.parachutes = [Parachute()] 
        self.corps = [Corps()]
        self.aileron = [Aileron()]
        self.ogive = Ogive() 
        self.jupe_retreints_list = [Jupe_retreint()]

        self.rcs = [RCS()]

        self.cn = None
        self.cg = None
        self.cpa = None
        accelerometres_list=[Accelerometre()]
        gyroscopes_list = [Gyroscope('zyx')] # La séquence des angles de cardant : 'zyx'
        barometres_list = None 
        magnetometres_list = None
        thermometres_list = None
        GPS_list = [GPS()]

        self.fusee = Fusee(self.aux_masse, self.corps, self.moteur, self.nb_aileron, self.aileron,
                           self.ogive, self.jupe_retreints_list, self.parachutes, self.rcs, self.cn, self.cg, self.cpa) 
        self.inertie = inertie(self.fusee)

############################################################################################################################
        
        
import matplotlib.pyplot as plt
plt.figure()
my_rocket = MyRocket()

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
