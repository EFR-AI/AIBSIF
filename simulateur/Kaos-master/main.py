import numpy as np
import time

from rockets.sera3 import sera3, affichage_fusée
from wind import ActiveWind 

from affichage import *
from quaternion import cardan_to_quaternion
from equation import trajectoire_6ddl
from result_table import result_sensors, results_table
from gps import Coordonne_gps
import random
from tqdm import tqdm

from rockets_builder import *
from engine_parser import *
from rockets.rocketclass import Rocket


for name in tqdm(range(100)):

    ###### Paramètres du vol qui peuvent être modifiés ###################

    # initialise la fusée, il faut associer le fichier de Cx équivalent qu'il faut importer
    my_rocket = Rocket(
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
        accelerometres_list=[Accelerometre([2.09,0.0,0.0],phi=0.0, theta=0.0, psi=0.0, incertitude_phi=0.0, incertitude_theta=0.0, incertitude_psi=0.0, noise_power=random.random()/100, erreur_de_justesse=random.randint(0,10)/100)],
        gyroscopes_list = [Gyroscope('zyx', noise_power=random.random()/10, erreur_de_justesse=random.randint(0,10)/100)],
        gyrometres_list = [Gyrometre(noise_power=random.random()/10, erreur_de_justesse=random.randint(0,10)/100)],
        barometres_list = None,
        magnetometres_list = [Magnetometre(noise_power=random.random()/10, erreur_de_justesse=random.randint(0,10)/100)],
        thermometres_list = None,
        GPS_list = [GPS(noise_power=random.random()/10, erreur_de_justesse=random.randint(0,10)/10)],
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


    # initialise le fichier de vent, il doit être situé dans le dossier wind    
    wind_arg = ActiveWind('dir_wind_profile2')


    # facteur multiplicatif de la norme du vent
    # mettre à 0 si on veut pas de vent, 1 pour avoir le fichier de vent original ou autre facteur
    puissance_vent = random.choice([0,1,2])


    # Parametre de la rampe de lancement 
    longeur_rampe = random.choice([7,8,9,10,11,12,13,14,15])                   # taille de la rampe de lancement
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
    nb_iter_max = 3000

    # Choisir si on veut faire du controle avec un ordinateur de bord ou non
    control = False #True si oui et False si non

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
    #affichage_parametre(t, list_v, "temps (s)", "vitesse (m.s-1)")
    #vent(wind_arg,10000,5)         
    #affichage_quiver(first_part,second_part,ind_apogee,list_x,list_y,list_z,list_u1,list_v1,list_w1,list_u2,list_v2,list_w2,list_u3,list_v3,list_w3)
    #affichage_arrow_dynamic(list_x, list_y, list_z, list_u1, list_v1, list_w1)
    #affichage_fusée(my_rocket)
    #plt.show()

    ##################### Fichier des données #################### 
    sequence = "zyx"    # "xyx", "yzy", "zxz", "xzx", "yxy", "zyz", "xyz", "yzx", "zxy", "xzy", "yxz", "zyx"
    periode_ech = 0.02
    gps = Coordonne_gps([latitude,longitude])
    results_table(str(name),t, Y, periode_ech, wind_arg, my_rocket.fusee.X_CG,gps, sequence, temps=1, z=1, accelerations_mesurées=1, angles_euler=1)
    result_sensors(str(name),my_rocket, t, periode_ech)
    del my_rocket
    