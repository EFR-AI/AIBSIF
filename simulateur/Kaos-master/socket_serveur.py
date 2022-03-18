## Prototype de socket développé pour le projet Perseus Avionique de l'Ecole Centrale de Lille
## Prototype de serveur

#######################
####### IMPORTS #######
#######################

import socket
import re
import time

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

#######################
###### TRANSFER #######
#######################

def until_first_backslashx00(string):
    """ Function that take a string containing \x00 and return all the
    string until the first \x00 found.
    
    Takes : 128\\x005\\x00
    Returns : 128"""

    return string.split('\x00', 1)[0]


number = 1

# TANT QUE : condition sur l'arrêt du programme serveur
while number < 2000000:
    # Méthode .send(ce_qu'on_veut) : permet d'envoyer ce_qu'on_veut par le port de connexion_avec_client
    number = str(number).encode() # On encode la string contenant les données à envoyer
    connexion_avec_client.send(number) # On envoie
    print("Nombre envoyé :", number, "\n")
    #time.sleep(4)

    # Méthode .recv(int) permet de recevoir les int premiers caractères reçu par le port de connexion_avec_client
    # On s'attend à recevoir 1024 caractères (modifiables)
    new_number = connexion_avec_client.recv(128) 
    print("On a reçu les bits :", new_number)

    new_number = new_number.decode() 

    new_number = until_first_backslashx00(new_number) 
    #les bits manquant sont marqué par \x00 qu'il faut retirer avant conversion
        
    new_number = float(new_number)
    print("On a reçu le nombre :", new_number)
    #time.sleep(4)

    ######### ON INSERE ICI LE CALCUL DU PROGRAMME SUR LES DONNEES RETOURNEES PAR L'ALGO DE CONTROLE #########
    
    number = new_number*4
    
    ######### ON INSERE ICI LE CALCUL DU PROGRAMME SUR LES DONNEES RETOURNEES PAR L'ALGO DE CONTROLE #########


#######################
######## CLOSE ########
#######################

print("On ferme la socket.")
connexion_avec_client.close()