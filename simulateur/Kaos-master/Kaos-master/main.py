import numpy as np
import time

from rockets.sera3 import SERA3, affichage_fusée
from wind import ActiveWind 

from affichage import *
from quaternion import cardan_to_quaternion
from equation import trajectoire_6ddl
from result_table import result_sensors, results_table
from gps import Coordonne_gps



###### Paramètres du vol qui peuvent être modifiés ###################

# initialise la fusée, il faut associer le fichier de Cx équivalent qu'il faut importer
my_rocket = SERA3    


# initialise le fichier de vent, il doit être situé dans le dossier wind    
wind_arg = ActiveWind('dir_wind_profile2')


# facteur multiplicatif de la norme du vent
# mettre à 0 si on veut pas de vent, 1 pour avoir le fichier de vent original ou autre facteur
puissance_vent = 2     
                        

# Parametre de la rampe de lancement 
longeur_rampe = 8                   # taille de la rampe de lancement
angle_rampe = 82 / 90 * np.pi / 2   # angle de la rampe avec le sol en radian 

#Site de lancement actuellemnt Kiruna
latitude  =  67.8557200
longitude =  20.2251300
altitude  = 0

# départ face au vent ou autre angle en radian 
beta =  -wind_arg.departure_wind_heading   


# pas d'intégration pour la méthode Runge kutta 4
h = 0.02           

# initialise le nombre d'iteration max
nb_iter_max = 300

# Choisir si on veut faire du controle avec un ordinateur de bord ou non
control = True #True si oui et False si non

# à exprimer en multiple du pas d'integration
delais_mvt_tuyere = 2

#####Initialisation Ne pas modifier sauf en connaissance de cause ####

param_rampe   = longeur_rampe, angle_rampe,(latitude,longitude,altitude)
param_vent    = wind_arg,puissance_vent
param_control = [control, delais_mvt_tuyere]

q0, q1, q2, q3 = cardan_to_quaternion(0, angle_rampe, beta)     
y0 = np.array([0, 0, -altitude, 0, 0, 0, q0, q1, q2, q3, 0, 0, 0])      

debut = time.time()     # initialise la mesure de la durée que met l'algorithme
t, Y, Apogee = trajectoire_6ddl(h, y0, nb_iter_max, my_rocket, param_vent, param_rampe, param_control) # Fonction Principale 

print("Durée de la simulation", time.time() - debut)


##### Paramètre d'affichage ##########################################

# Pour tracer les courbes, on peut choisir de ne tracer qu'une partie des points uniquement

# echantillonage des points : 1 tous les points, 2 un point sur 2 etc ...
n = 1 

# la montée etant plus rapide que la descente pour affichage_quiver() on peut differencier 
# l'echantillonage pour un meilleur affichage
# first_part  : echantillonage de la montée 
# second_part : echantillonage de la descente  

if h <= 0.02 : 
    first_part = 50    
    second_part = 1000   
elif h <= 0.05 : 
    first_part = 20    
    second_part = 200
else : 
    first_part = 10    
    second_part = 100


######Formatage à ne pas modifier ####################################
#print(Y[0])
#Y = np.array(Y[:-10])
#print(Y)
Y = np.array(Y)
list_z = -Y[0 : len(Y) : n, 2]
list_x = Y[0 : len(Y) : n, 0]
list_y = Y[0 : len(Y) : n, 1]
list_v = np.sqrt(Y[0 : len(Y) : n, 3] ** 2 + Y[0 : len(Y) : n, 4] ** 2 + Y[0 : len(Y) : n, 5] ** 2)
t = np.linspace(0, len(Y), len(Y) // n)*h


norme = np.sqrt(max(list_x) ** 2 + max(list_y) ** 2 + max(list_z) ** 2)
aggrandissement = 300

list_u1, list_v1, list_w1 = format_vecteurs(Y, n, norme, (1, 0, 0))
list_u2, list_v2, list_w2 = format_vecteurs(Y, n, norme, (0, 1, 0))
list_u3, list_v3, list_w3 = format_vecteurs(Y, n, norme, (0, 0, 1))

list_phi, list_theta, list_psi = format_angle_quaternion(Y, echantillonage=n)
list_vent = np.array([wind_arg.return_wind(h) for h in list_z])


M = max(list_z)
ind_apogee = np.where(list_z == M)[0][0]


#####################Affichage########################################
# Decommenter la ligne pour avoir ce que vous voulez
######################################################################

#affichage_parametre            : affiche 2 paramètre l'un en fonction de l'autre. Les parametre disponibles sont :
#                                 temps t, positions x,y,z avec list_x,list_y,list_z, norme de la vitesse avec list_v,
#                                 les angles de cardan avec list_phi,list_theta,list_psi. 
#                                 Tout autre paramètre est a implémenté soi même. 
#affichage_angle_quaternion     : affiche les angles de cardan en fonction de l'altitude ainsi qu'une composante du vent, 
#                                 permet d'etudier la stabilité
#vent                           : affiche la colonne de vent pour l'altitude donnée
#affichage_quiver               : affiche la trajectoire en 3d sous forme de vecteur
#affichage_arrow_dynamic        : affiche la trajectoire en 3d dynamique 
#affichage_fusée                : affiche le modèle de la fusée


#affichage_angle_quaternion(list_z,list_phi,list_theta,list_psi,list_vent)

#affichage_parametre(t, list_v, "temps (s)", "attitude (rd)")
#affichage_parametre(t, list_theta, "temps (s)", "attitude (rd)")
affichage_parametre(t, list_v, "temps (s)", "vitesse (m.s-1)")
#vent(wind_arg,10000,5)         
#affichage_quiver(first_part,second_part,ind_apogee,list_x,list_y,list_z,list_u1,list_v1,list_w1,list_u2,list_v2,list_w2,list_u3,list_v3,list_w3)
#affichage_arrow_dynamic(list_x, list_y, list_z, list_u1, list_v1, list_w1)
affichage_fusée(my_rocket)
plt.show()

##################### Fichier des données #################### 
sequence = "zyx"    # "xyx", "yzy", "zxz", "xzx", "yxy", "zyz", "xyz", "yzx", "zxy", "xzy", "yxz", "zyx"
periode_ech = 0.02
gps = Coordonne_gps([latitude,longitude])
results_table (t, Y, periode_ech, wind_arg, my_rocket.fusee.X_CG,gps, sequence, temps=1, z=1, accelerations_mesurées=1, angles_euler=1)
result_sensors(my_rocket, t)