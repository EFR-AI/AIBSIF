from os import X_OK
import socket
from typing import FrozenSet
from numpy import cos, sin, arcsin, arctan
import numpy.linalg as linalg
import numpy as np

from fonction_interpolation import interpolation
from g_variable import g_acceleration_pesanteur
from quaternion import Mat_deriv_quaternion,Mat_fusee_terre
from control import Controler
from gps import Coordonne_gps
from inertia_calculation import matrice_inertie_fusée
from rockets.rocketclass import Rocket
from thermodynamic_model import Pression,Temperature,rho_air
from traitement_message import decodage_commande, encodage_mesure, until_first_backslashx00


"""
def Vect_vent(X,puissance_vent):
    
    #paramètre : vecteur X = (x,y,z,vx,vy,vz,q0,q1,q2,q3,p,q,r)
    #renvoie le vecteur du vent pour l'instant vent horizontal à la surface de la terre
    
    Mat_terre_vent = np.array(
        [[cos(eps), -sin(eps), 0], [sin(eps), cos(eps), 0], [0, 0, 1]]
    )

    return Mat_fusee_terre(X).dot(
        linalg.inv(Mat_terre_vent).dot(np.array([puissance_vent, 0, 0]))
    )
"""

def Vitesse_relative(X, V_abs, puissance_vent, Vect_vent):
    """
    paramètre : 
    - vecteur X = (x,y,z,vx,vy,vz,q0,q1,q2,q3,p,q,r) : le vecteur état de la fusée
    
    Retourne : Vitesse relative de la fusee : vitesse de la fusee - vitesse vent
    """
    Vent = Mat_fusee_terre(X).dot(Vect_vent) * puissance_vent

    return -np.array([-V_abs[0] + Vent[0], -V_abs[1] + Vent[1], -V_abs[2] + Vent[2]])


def Force_Poussee(P, deltr, deltq):
    """
    Paramètre : 
    - P : la norme de la force de Poussée
    - deltr, deltq : angles de braquage de la tuyère
    
    Retourne : le vecteur de poussée du moteur dans le repère de la fusée
    """
    M = np.array(
        [
            [cos(deltr) * cos(deltq)    , -sin(deltr)   , cos(deltr) * sin(deltq)],
            [sin(deltr) * cos(deltq)    , cos(deltr)    , sin(deltr) * sin(deltq)],
            [-sin(deltq)                , 0             , cos(deltq)             ],
        ]
    )
    return np.dot(M, np.array([P, 0, 0]))


def Poid(masse, X, gps):
    """
    Paramètres : 
    - masse : la masse de la fusée
    - Vecteur X = (x,y,z,vx,vy,vz,q0,q1,q2,q3,p,q,r) : le vecteur état de la fusée
    - gps : l'object permettant de calculer la position gps de la fusée
    
    Retourne : le vecteur de la force du poids dans le repère de la fusée
    """
    lat,lon,alt = gps.position_GPS(X)
    g = g_acceleration_pesanteur(lat, alt)
    P = np.array(
        [0, 0, masse * g],
    )
    return np.dot(Mat_fusee_terre(X), P)


def Reaction_support_rampe(masse, angle_rampe, X, gps):
    """
    Paramètres : 
    - masse : la masse de la fusée
    - angle_rampe : l'angle de la rampe de lancement par rapport à la normal au sol
    - Vecteur X = (x,y,z,vx,vy,vz,q0,q1,q2,q3,p,q,r) : le vecteur état de la fusée
    - gps : l'object permettant de calculer la position gps de la fusée

    Retourne : le vecteur de la force de réaction du support dans le repère de la fusée
    """
    lat,lon,alt = gps.position_GPS(X)
    g = g_acceleration_pesanteur(lat,alt)
    
    R = np.array([0, 0, -masse * g * cos(angle_rampe)])
    return np.dot(Mat_fusee_terre(X), R)


def Cx_v(list_Cx, V, z):
    """
    Paramètres : 
    - list_Cx : tableau de l'évolution du coefficient de trainé (Cx) en fonction du nombre de Mach
    - V : la norme de la vitesse
    - z : l'altitude
    
    L'évolution du Cx selon la vitesse est tiré de l'abaque du doc du cnes.
    Utilisation d'une interpolation linéaire

    Retourne le Cx de la fusée
    """
    T = Temperature(z)
    c = np.sqrt(1.403 * 287 * T)  # vitesse du son
    Mach = V / c
    return interpolation(list_Cx, Mach)




def Force_aero(V, rho, C_A, Cn_alpha, Surface):
    """
    Paramètres : 
    - V : vecteur vitesse
    - rho : densite de l'air 
    - CX0 : coefficient de trainée
    - Cn_alpha : le gradient de portance
    - Surface : 
    
    Retourne : la force aerodynamique : Vecteur Portance et Traine
    """

    Norme = linalg.norm(V)

    # pour éviter de calculer des valeurs alpha et beta incongrues
    if linalg.norm(V) <= pow(10, -10):
        return np.zeros(3)
    else:
        alpha = arctan(V[2] / V[0])
        beta = arctan(V[1] / Norme)
        C_Y = Cn_alpha * beta
        C_N = Cn_alpha * alpha
        #print(C_Y,C_N,V)
        return (
            -1 / 2 * rho * Surface *  Norme ** 2 * np.array([ C_A, C_Y, C_N]) 
        )


def Moment_aero(X, V, rho, Cn_alpha, coeff, Surface, l_ref, l_d):
    """
    Paramètres :
    - X : Vecteur état de la fusée
    - V : Vecteur vitesse
    - rho : densite de l'air
    #Force aerodynamique : Portance et Traine
    #Moment des Forces aerodynamique
    """
    p,q,r = X[-3:]
    Norme = linalg.norm(V)
    C_l0, C_lp, C_mq, C_nr = coeff

    # pour éviter de calculer des valeurs alpha et beta incongrues
    if linalg.norm(V) <= pow(10, -10):
        return np.zeros(3)
    else:
        alpha = arctan(V[2] / V[0])
        beta = arctan(V[1] / Norme)
        C_malpha = Cn_alpha * alpha
        C_nbeta = -Cn_alpha * beta

    return (
            -1 / 2 * rho * Surface *  Norme ** 2 *  np.array([ l_d*C_l0     - p/(2*Norme)*l_d**2*C_lp, 
                                                            l_ref*C_malpha - q/(2*Norme)*l_d**2*C_mq, 
                                                            l_ref*C_nbeta  - r/(2*Norme)*l_d**2*C_nr]) 
        )


def Moment_poussee(P, deltr, deltq, distance_tuyere_cg):
    """
    paramètre : P norme de la pousse, distance_tuyere_cg = distance entre le bas de la tuyere et le centre le gravité
    Moment des Forces de poussee
    """
    return np.cross(distance_tuyere_cg * np.array([-1, 0, 0]), Force_Poussee(P, deltr, deltq))


def F_rampe(X, t, Parameters):
    """
    Tant que la fusée est sur la rampe : on rajoute la réaction du support ainsi que celle du sol
    F de l'équation differentielle :
    y' = F(t,y)
    avec y = (x,y,z,vx,vy,vz,q0,q1,q2,q3,p,q,r)
    """
    ub = X[3]
    Vfusee = np.array([ub, 0, 0])

    [angle_rampe, masse, dm, Poussee,gps] = Parameters[:5]
    # PFD dans un référentiel non galiléen
    Acceleration = ( 1 / masse * 
        (
            Poid(masse, X,gps)
            + Force_Poussee(Poussee,0,0)
            + Reaction_support_rampe(masse, angle_rampe, X,gps)
            - dm * Vfusee
        )
    )

    dub = Acceleration[0]

    #reaction du sol 
    if dub < 0:
        dub = 0

    [dx, dy, dz] = np.transpose(Mat_fusee_terre(X)).dot(Vfusee)

    return np.array([dx, dy, dz, dub, 0, 0, 0, 0, 0, 0, 0, 0, 0])


def F_air(X, t, Parameters):
    """
    mouvement de la fusée dans les airs
    F de l'équation differentielle :
    y' = F(t,y)
    Paramètres :
    - X = (x,y,z,vx,vy,vz,q0,q1,q2,q3,p,q,r) : le vecteur état de la fusée
    - t : le temps
    - Parameters : parametres physiques =  [angle_rampe, masse, dm, Poussee, gps, Cx, Cn_alpha, Coeff, Distance, Surface, rho, Inertie, Inv_inertie, puissance_vent, Vect_vent, deltr, deltq]
    """

    [ub, vb, wb, q0, q1, q2, q3, p, q, r] = X[3:]

    #Normalisation des quaternions pour la stabilité 
    N = q0 ** 2 + q1 ** 2 + q2 ** 2 + q3 ** 2
    q0, q1, q2, q3 = q0 / N, q1 / N, q2 / N, q3 / N

    #print([q0, q1, q2, q3])

    [
        angle_rampe, 
        masse,
        dm,
        Poussee,
        gps,
        Cx,
        Cn_alpha,
        Coeff,
        Distance,
        Surface,
        rho,
        Inertie,
        Inv_inertie,
        puissance_vent,
        Vect_vent,
        deltr,
        deltq
    ] = Parameters
    
    [l_d,l_ref,l_p,l_o_xcg] = Distance
    # Calcul de la vitesse relative
    Vfusee = np.array([ub, vb, wb])
    V = Vitesse_relative(X, Vfusee,puissance_vent, Vect_vent)
       

    Mat_rot = np.array(
        [[0, -r,  q], 
        [r,   0, -p], 
        [-q,  p, 0]],
    )

    Mat_deriv = Mat_deriv_quaternion(X)

    [dq0, dq1, dq2, dq3] = 1 / 2 * Mat_deriv.dot(np.array([q0, q1, q2, q3]))
    
    F_aero = Force_aero(V, rho, Cx, Cn_alpha, Surface)
    F_Poussee = Force_Poussee(Poussee,deltr,deltq)
    # PFD dans un référentiel non galiléen
    Acceleration = 1 / masse * (
        Poid(masse, X,gps)
        + F_Poussee
        + F_aero
        - dm * Vfusee
    ) - Mat_rot.dot(Vfusee)

    [dub, dvb, dwb] = Acceleration

    # Centrage par varignon
    M_aero = Moment_aero(X,V, rho, Cn_alpha,Coeff, Surface, l_ref,l_d) + np.cross(l_o_xcg*np.array([-1,0,0]),F_aero)
    M_Pousse = Moment_poussee(Poussee, deltr, deltq,l_p)+ np.cross(l_o_xcg*np.array([-1,0,0]),F_Poussee)
    
    # théorème du moment cinétique dans un référentiel non galiléen
    Moments = (
        M_aero
        + M_Pousse
        - Mat_rot.dot(Inertie).dot(np.array([p, q, r]))
        - dm * Inertie.dot(np.array([p, q, r]))
    )
    
    #print("M:",Moment_poussee(Poussee, deltr, deltq, distance_tuyere_cg))
    [dp, dq, dr] = Inv_inertie.dot(Moments)

    [dx, dy, dz] = np.transpose(Mat_fusee_terre(X)).dot(Vfusee)
    res = np.array([dx, dy, dz, dub, dvb, dwb, dq0, dq1, dq2, dq3, dp, dq, dr])

    # utile pour les vols strictement verticaux 
    # cos(pi/2) != 0, pour avoir une fusée parfaitement verticale sinon l'erreur se propage
    res[abs(res) < pow(10, -10)] = 0  
   
    return res


def Integration_Runge_Kutta(f, y, t, h, Parameters):
    """
    Méthode d'intégration numérique Runge Kutta
    Paramètres : 
    - f : fonction de l'équa dif y' = f(t,y)
    - y = (x,y,z,vx,vy,vz,q0,q1,q2,q3,p,q,r) : le vecteur état de la fusée
    - t : le temps
    - h : le pas d'intégration
    - Parameters : parametre physique necessaire à f

    Retourne : y à t+h
    """

    k1 = f(y, t, Parameters)
    k2 = f(y + h / 2 * k1, t + h / 2, Parameters)
    k3 = f(y + h / 2 * k2, t + h / 2, Parameters)
    k4 = f(y + h * k3, t, Parameters)

    return y + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


def Integration_Euler(f, y, t, h, Parameters):
    """
    Méthode d'intégration numérique Euler
    Paramètres : 
    - f : fonction de l'équa dif y' = f(t,y)
    - y = (x,y,z,vx,vy,vz,q0,q1,q2,q3,p,q,r) : le vecteur état de la fusée
    - t : le temps
    - h : le pas d'intégration
    - Parameters : parametre physique necessaire à f

    Retourne : y à t+h
    """
    return y + h * f(y, t, Parameters)


def Courbe_Poussee(t, masse, Masse_vide, Masse_ergol, Tableau_poussee ):
    """
    evolution de la norme de la force de poussee et de la masse au cours du temps
    paramètre t temps, h pas d'integration, masse à vide, masse totale d'ergol
    """
    Poussee = interpolation(Tableau_poussee, t)
    new_masse = interpolation(
        np.array([[0, Masse_vide + Masse_ergol], [Tableau_poussee[-1, 0], Masse_vide]]),
        t,
    )
    dm = masse - new_masse

    return new_masse, dm, Poussee


def trajectoire_6ddl(h, y0, nb_iter_max, my_rocket:Rocket, param_vent, param_rampe, param_control):
    """
    parametre h pas d'integration, vecteur initial y0 = (x,y,z,vx,vy,vz,q0,q1,q2,q3,p,q,r),
    retourne liste des temps, liste de vecteur y = (x,y,z,vx,vy,vz,q0,q1,q2,q3,p,q,r)
    """

    # parametres 
    Longeur_rampe, angle_rampe,localisation = param_rampe
    wind_arg,puissance_vent = param_vent
    
    # Dimension
    Hauteur = my_rocket.fusee.hauteur  # conversion en m
    Diametre = 0
    Dmax = 0
    i=0
    for corps in my_rocket.corps:
        i += 1
        Diametre += corps.d_corps  # conversion en m
        if Dmax<corps.d_corps:
            Dmax = corps.d_corps
    Diametre = Diametre/i

    # Masse
    Masse_vide = my_rocket.fusee.masse_vide
    Masse_ergol = my_rocket.fusee.masse_ergol
    masse = Masse_ergol + Masse_vide  # totale

    # Parametre aerodynamique
    Cn_alpha = my_rocket.fusee.Cn_alpha
    X_CPA = my_rocket.fusee.X_CPA  # conversion en m
    X_CG = my_rocket.fusee.X_CG  # my_rocket.fusee.X_CG/100  # conversion en m
    X_O = my_rocket.fusee.X_O
    Coeff = my_rocket.fusee.Coeff
    Distance = Diametre, X_CPA - X_O, Hauteur - X_O, X_O - X_CG

    # Parachutes :
    Liste_parachutes = my_rocket.parachutes[:]

    Surface = np.pi * Diametre ** 2 / 4
    #Surface = np.pi * Dmax ** 2 / 4
    Tableau_poussee = np.array(my_rocket.moteur.thrust)
    Temps_poussee = my_rocket.moteur.thrust[-2][0]

    # Vent
    wind_function = wind_arg.return_wind

    # Site de lancement 
    gps = Coordonne_gps(localisation)

    # Controleur
    control = param_control[0]
    delais_mvt_tuyere = param_control[1]

    if control:
        # Initialisation du dialogue

        #######################
        #### CONSTRUCTION #####
        #######################

        # AF : Adress Family (Inet = Internet) à explorer
        # Sock_stream : pour protocole TCP (protocole de transfert sûr ≠ UDP rapide mais transfert peu sûr)
        connexion_principale = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Méthode .bind((nom_d'hote ; port_à_écouter)) : laisser le nom d'hôte vide pour attendre la connection
        # Il faut que ce soit un tuple, port entre 1024 et 65535 (1023 premier pour le système)
        port_number = 26000
        connexion_principale.bind(('',port_number))
        print("Opening connection at port :", port_number)

        # Méthode .listen(int) : nombre de client refusé avant d'arrêter d'écouter 
        connexion_principale.listen(5)
        print("Listenning")

        # Méthode .accept() : renvoie le tuple (socket_pour_dialogue_avec_le_client, (adresse_IP_client, port_de_connexion))
        print("Waiting ...")
        connexion_avec_client, infos_connexions = connexion_principale.accept()
        print("Connection accepted. IP :", infos_connexions[0], "Exit port :", infos_connexions[1])

    # Définition de la matrice d'inertie de la fusée
    """
    Ix, Iy, Iz = (
        (Diametre / 2) ** 2 / 2,
        (Diametre / 2) ** 2 / 4 + 1 / 12 * Hauteur ** 2,
        (Diametre / 2) ** 2 / 4 + 1 / 12 * Hauteur ** 2,
    )
    Inertie = masse * np.array([[Ix, 0, 0], [0, Iy, 0], [0, 0, Iz]])
    Inv_inertie = 1 / masse * np.array([[1 / Ix, 0, 0], [0, 1 / Iy, 0], [0, 0, 1 / Iz]])
    """
    deltq,deltr = 0 ,0
    commande_deltq,commande_deltr = 0,0 
    deltq0,deltr0 = 0,0

    
    Inertie = matrice_inertie_fusée(my_rocket,deltq,deltr)
    Inv_inertie = linalg.inv(Inertie)

    # Initialisation
    t = [0]
    y = [y0]
    Apogee = -y0[2]
    i,i0 = 0,0
    f = F_rampe

    # Parametre pour générer les fichiers de test necessaire au pole attitude 

    #Liste_gravite = [Poid(1, y0)]
    #Bh, Bv = 23, 39.84 # valeur à Montpellier pour le pole attitude de Perseus Centrale Lille
    #Liste_champs_magnetique = [np.dot(Mat_fusee_terre(y0), np.array([Bh, 0, Bv]))]

    print("Temps de Poussee de la fusée :", Temps_poussee)

    while -y[-1][2] >= -y0[2] and i <= nb_iter_max:

        i += 1 
        
        
        # actualisation de la masse, force de Poussee, de la densité de l'air et du vent
        masse, dm, Poussee = Courbe_Poussee(
            i * h, masse, Masse_vide, Masse_ergol, Tableau_poussee
        )
        rho = rho_air(-y[-1][2])
        Vect_vent = wind_function(-y[-1][2])

        # test de sortie de rampe
        if f == F_rampe and -y[-1][2] > Longeur_rampe * sin(angle_rampe):
            print("vitesse sortie de rampe", y[-1][3])
            print("temps écoulé à la sortie de rampe", i*h)
            f = F_air
        else : 

            for k in range(len(Liste_parachutes)):
                if Liste_parachutes[k].apogee:
                    altitude_deploiment = (
                        Apogee - 1
                    )  # simule le retard de déploiement, faudrait peut être mieux l'indexer
                else:
                    altitude_deploiment = Liste_parachutes[k].alt

                Liste_Surface = [Surface]
                Liste_Cx = [Cx_v(my_rocket.list_Cx, linalg.norm(y[-1][3:6]),-y[-1][2] )]
                if (
                    np.transpose(Mat_fusee_terre(y[-1]))
                    .dot(np.array([1, 0, 0]))
                    .dot(np.array([0, 0, 1]))
                    >= 0
                    and -y[-1][2] <= altitude_deploiment
                ):
                    Liste_Cx.append(1.5)

                    Liste_Surface.append(Liste_parachutes[k].surface)
                Surface = max(Liste_Surface)
                Cx = max(Liste_Cx)
                
        #"""
        # Pour arreter la simulation du vol à son apogée
        # permet d'éviter de calculer la descente très gourmande en calcul 
        # pour les tests, à décommenter si besoin 
        
        # if (
        #     np.transpose(Mat_fusee_terre(y[-1]))
        #     .dot(np.array([1, 0, 0]))
        #     .dot(np.array([0, 0, 1]))
        #     >= 0
        #     and -y[-1][2] <= Apogee - 0.2
        # ):
        #     break
        #"""
        

        Parameters = [
                angle_rampe, 
                masse,
                dm,
                Poussee,
                gps,
                Cx,
                Cn_alpha,
                Coeff,
                Distance,
                Surface,
                rho,
                Inertie,
                Inv_inertie,
                puissance_vent,
                Vect_vent,
                deltr,
                deltq,
                
            ]

        # Mesure
        sensor_types = {
            'accelerometres' : my_rocket.fusee.accelerometres_list,
            'gyroscopes' : my_rocket.fusee.gyroscopes_list,
            'barometres' : my_rocket.fusee.barometres_list,
            'thermometres' : my_rocket.fusee.thermometres_list,
            'magnetometres' : my_rocket.fusee.magnetometres_list,
            'GPS' : my_rocket.fusee.GPS_list
        }

        payload  = measureForPayload(sensor_types, y, gps, h, X_CG)
        
        message = messageFromPayload(payload)
        
        Fonction_mesure(my_rocket, y, gps, h, X_CG)
        
        if control:
            # Appel au controleur 
            message = encodage_mesure(i, y[-1], Parameters)
            message = message.encode() # On encode la string contenant les données à envoyer
            connexion_avec_client.send(message) # On envoie
            #print("Nombre envoyé :", message, "\n")

            new_message = connexion_avec_client.recv(128) 
            #print("On a reçu les bits :", new_message)

            new_message = new_message.decode() 

            new_message = until_first_backslashx00(new_message) 
            #les bits manquant sont marqué par \x00 qu'il faut retirer avant conversion
            
            answer, correction = decodage_commande(new_message)
            # print("On a reçu le nombre :", new_message)
            # print(answer)
            # print(correction)

            ######### ON INSERE ICI LE CALCUL DU PROGRAMME SUR LES DONNEES RETOURNEES PAR L'ALGO DE CONTROLE #########
    

            if answer :
                #print("actif",i)
                deltq0,deltr0 = deltq,deltr
                commande_deltq,commande_deltr = correction
                i0 = i 

                     
        deltq = interpolation(np.array([[i0,deltq0],[i0+delais_mvt_tuyere,commande_deltq]]),i)  
        deltr = interpolation(np.array([[i0,deltr0],[i0+delais_mvt_tuyere,commande_deltr]]),i)  

        
        Inertie = matrice_inertie_fusée(my_rocket,deltq,deltr)
        Inv_inertie = linalg.inv(Inertie)  

        Parameters[-5] = Inv_inertie
        Parameters[-6] = Inertie 
        Parameters[-2] = deltr
        Parameters[-1] = deltq

        
  
        # Integration

        new = Integration_Runge_Kutta(f, y[-1], t[-1], h, Parameters)
        y.append(new)
        t.append(i * h)
        Apogee = max(-y[-1][2], Apogee)
    
    Fonction_mesure(my_rocket, y, gps, h, X_CG)
    if control : 
        # On termine la communication avec l'ordinateur de bord
        print("On ferme la socket.")
        connexion_avec_client.close()


    # Affichage des caractéristiques de Vol et de stabilité
    print("Apogee", Apogee)
    print("Finesse", Hauteur / Diametre)
    print("Marge Statique", (X_CPA-X_CG) / Diametre, "calibres")
    print("Cnalpha", Cn_alpha)
    print("produit Cn_alpha*MS", Cn_alpha * (X_CPA-X_CG))
    print("Masse à vide", Masse_vide, "Masse initiale", Masse_vide + Masse_ergol)
    print("Nb iteration", i)
    print("Position Finale, Lat Long alt : ", gps.position_GPS( y[-2][:3]),"\nEn m depuis le point de départ : ",y[-2][:3])
    return t, y, Apogee

def Fonction_mesure(my_rocket, y, gps, h, X_CG):
    mesures = []

    for capteur in my_rocket.fusee.accelerometres_list:
        if len(y)>2:
            mesures.append(capteur.mesure(y[-1], y[-2], y[-3], gps, h, X_CG))
        elif len(y)>1:
            mesures.append(capteur.mesure(y[-1], y[-2], None, gps, h, X_CG))
        else:
            mesures.append(capteur.mesure(y[-1], None, None, gps, h, X_CG))
        
    for capteur in my_rocket.fusee.gyroscopes_list:
        mesures.append(capteur.mesure(y[-1]))

    for capteur in my_rocket.fusee.gyrometres_list:
        if len(y)>1:
            mesures.append(capteur.mesure(y[-1], y[-2], h))
        else:
            mesures.append(capteur.mesure(y[-1], None, h))
        

    for capteur in my_rocket.fusee.barometres_list:
        mesures.append(capteur.mesure(y[-1]))

    for capteur in my_rocket.fusee.thermometres_list:
        mesures.append(capteur.mesure(y[-1]))
        
    for capteur in my_rocket.fusee.magnetometres_list:
        mesure = capteur.mesure(y[-1], gps, fusee=False) 
        mesures.append(mesure)

    for capteur in my_rocket.fusee.GPS_list:
        mesures.append(capteur.mesure(y[-1], gps))
    
    return mesures

def measureForPayload(sensor_types, y, gps, h, X_CG):
    payload  = {}
    for name, sensors in sensor_types.items():
        if sensors:
            if name != 'accelerometres':
                if name != 'magnetometres':
                    if name != 'GPS':
                        payload.update({f'{name}{i}' : sensor.mesure(y[-1]) for i, sensor in enumerate(sensors) })
                    else :
                        payload.update({f'{name}{i}' : sensor.mesure(y[-1], gps) for i, sensor in enumerate(sensors) })
                else :
                    payload.update({f'{name}{i}' : sensor.mesure(y[-1], gps, fusee=False) for i, sensor in enumerate(sensors) })
            else :
                if len(y)>2:
                    payload.update({f'{name}{i}' : sensor.mesure(y[-1], y[-2], y[-3], gps, h, X_CG) for i, sensor in enumerate(sensors) })
                elif len(y)>1:
                    payload.update({f'{name}{i}' : sensor.mesure(y[-1], y[-2], None, gps, h, X_CG) for i, sensor in enumerate(sensors) })
                else :
                    payload.update({f'{name}{i}' : sensor.mesure(y[-1], None, None, gps, h, X_CG) for i, sensor in enumerate(sensors) })
                    
    return(payload)

def messageFromPayload(payload):
    message = ''
    
    return message