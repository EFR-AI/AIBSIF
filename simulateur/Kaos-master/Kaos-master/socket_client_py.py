## Prototype de socket développé pour le projet Perseus Avionique de l'Ecole Centrale de Lille
## Prototype de client 

###### Controle d'attitude ###########################################
# Controle de l'orientation de la tuyère 
# Controle des RCS à implémenter 

# La fonction de controle actuellement implémentée n'est pas destinée 
# à être celle utilisée dans la fusée. Elle sert d'exemple pour tester 
# que l'on peut bien controler la fusée dans ce simulateur et 
# d'exemple pour l'implémentation d'autre algorithme de controle. 
# 
# La fonction actuelle permet de controler le quaternion complet et 
# pas uniquement l'angle theta par rapport au sol 

#######################
####### IMPORTS #######
#######################

import socket

import numpy as np
from wind import ActiveWind 
from quaternion import cardan_to_quaternion
from control import Controler
from traitement_message import decodage_commande, decodage_mesure,encodage_mesure, encodage_commande

#######################
#### CONSTRUCTION #####
#######################


# AF : Adress Family (Inet = Internet) à explorer
# Sock_stream : pour protocole TCP (protocole de transfert sûr ≠ UDP rapide mais transfert peu sûr)
connexion_avec_serveur = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Méthode .connect(('nom_de_l'hôte', int_du_port))
# Attention au tuple
# On utilise localhost dans le cas où on écoute sur la même machine
connexion_avec_serveur.connect(('localhost', 26000))

########################################
###### Initialisation du control #######
########################################

# initialise le fichier de vent, il doit être situé dans le dossier wind    
wind_arg = ActiveWind('dir_wind_profile2')


# facteur multiplicatif de la norme du vent
# mettre à 0 si on veut pas de vent, 1 pour avoir le fichier de vent original ou autre facteur
puissance_vent = 0.1      
                        

# Parametre de la rampe de lancement 
longeur_rampe = 8                   # taille de la rampe de lancement
angle_rampe = 82 / 90 * np.pi / 2   # angle de la rampe avec le sol en radian 

#Site de lancement actuellemnt Kiruna
latitude  =  67.8557200
longitude =  20.2251300
altitude  = 0

# départ face au vent ou autre angle en radian 
beta =  -wind_arg.departure_wind_heading 

# à exprimer en multiple du pas d'integration
Temps_reponse_controleur  = 4 
delais_mvt_tuyere = 2

# la commande : c'est au choix un quaternion constant 
# ou l'angle theta constant, par défaut celui en theta 
# à modifier directement dans le fichier control
# peut être remplacé par une fonction qui évolue au cours du temps 
# pour faire de la navigation. 

quaternion_desire = cardan_to_quaternion(0,angle_rampe,beta) 

# debattement maximal de la tuyere en degre 
debattement = 7 

param_control = True,Temps_reponse_controleur,quaternion_desire,debattement,delais_mvt_tuyere
controle_tuyere = Controler(param_control[1],param_control[2],param_control[3])
delais_mvt_tuyere = param_control[4]

#######################
###### TRANSFER #######
#######################

commande = 0
Poussee = 1
while Poussee > 0:
    # Méthode .recv(int) permet de recevoir les int premiers caractères reçu par le port de connexion_avec_serveur
    # On s'attend à recevoir 1024 caractères (modifiables)
    message = connexion_avec_serveur.recv(1024)
    #print("Nombre reçu :", message)
    message = message.decode()
    #print(message)
    if decodage_mesure(message)==0: break
    i, X, Parameters = decodage_mesure(message)
    answer, correction = controle_tuyere.reception(i,X,Parameters)
    print(correction)
    print(answer)
    # Méthode .send(ce_qu'on_veut) : permet d'envoyer ce_qu'on_veut par le port de connexion_avec_serveur
    commande = encodage_commande(answer,correction)
    connexion_avec_serveur.send(commande.encode())
    print("On renvoit donc :", commande)
    

print("Fin de la communication et de la simulation\n")


#######################
######## CLOSE ########
#######################

connexion_avec_serveur.close()