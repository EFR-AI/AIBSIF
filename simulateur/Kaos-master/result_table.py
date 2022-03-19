import subprocess, sys
import numpy as np
from numpy.lib.function_base import append
from quaternion import quat2eul, singularitycheck, Mat_fusee_terre, quaternion_to_cardan
from affichage import format_angle_quaternion
from wind import ActiveWind 
from fonction_interpolation import interpolation
from g_variable import g_acceleration_pesanteur
from equation import Pression, Temperature



def results_table (name,t, Y, periode_ech, wind_arg, X_CG, gps, sequence, temps=None, x=None, y=None, z=None, vitesses=None, norme_vitesse=None, accelerations=None, accelerations_mesurées=None, angles_euler=None, angles_cardan=None, quaternions=None, vitesses_angulaires=None, accelerations_angulaires=None, pression=None, temperature=None, magnetique=None, GPS=None):
    """
    Paramètres : 
    - t : la liste des temps,
    - Y : l'historique de tous les états de la fusée pendant le vol
    - periode_ech : la période d'échantillonnage souhaitée (en secondes)
    - X_CG : la position du centre de gravité par rapport au sommet de la fusée
    - gps : l'object permettant de calculer la position gps de la fusée
    - sequence : le sequence de rotation souhaitée pour vos angles d'Euler (cf documentation et/ou main.py)
    Les autres paramètres sont à fixer à 1 pour avoir leur données dans le tableau

    La fonction produit fichier nommé "resultats.csv" contenant un tableau avec les données souhaitées.
    Ce fichier se trouve dans le dossier "resultats".

    /!\ n'oubliez pas de fermer le fichier avant de relancer la simulation, sinon il ne sera pas mis à jour /!\
    """
    
    ##### Récupération et génération des données
    # les données du milieu ambiant (champ magnétique, température et pression) sont générées après l'échantillonnage

    n = 1
    
    ### le temps
    list_t = np.array([t])

    dt = t[1]-t[0] # on enregistre le pas de temps car il sera utile pour la dérivation

    ### les positions
    list_x = np.array([Y[0 : len(Y) : n, 0]])
    list_y = np.array([Y[0 : len(Y) : n, 1]])
    list_z = np.array([-Y[0 : len(Y) : n, 2]])

    ### les vitesses
    list_vx = np.array([Y[0 : len(Y) : n, 3]])
    list_vy = np.array([Y[0 : len(Y) : n, 4]])
    list_vz = np.array([Y[0 : len(Y) : n, 5]])
    list_v = np.array([np.sqrt(Y[0 : len(Y) : n, 3] ** 2 + Y[0 : len(Y) : n, 4] ** 2 + Y[0 : len(Y) : n, 5] ** 2)])
    
    ### les angles de cardan
    list_x_cardan, list_y_cardan, list_z_cardan = format_angle_quaternion(Y, n)
    list_x_cardan, list_y_cardan, list_z_cardan = np.array([list_x_cardan]), np.array([list_y_cardan]), np.array([list_z_cardan])
    
    ### les quaternions
    list_q0 = np.array([Y[0 : len(Y) : n, 6]])
    list_q1 = np.array([Y[0 : len(Y) : n, 7]])
    list_q2 = np.array([Y[0 : len(Y) : n, 8]])
    list_q3 = np.array([Y[0 : len(Y) : n, 9]])
    
    ### les angles d'euler
    list_phi, list_theta, list_psi = angles_de_euler(sequence, list_q0, list_q1, list_q2, list_q3)

    ### le vent
    #list_vent = np.array([[wind_arg.return_wind(h) for h in list_z]])
    
    ### les dérivées premières et secondes des angles de cardan et d'Euler
    list_dx_cardan = np.array([Y[0 : len(Y) : n, 10]])
    list_dy_cardan = np.array([Y[0 : len(Y) : n, 11]])
    list_dz_cardan = np.array([Y[0 : len(Y) : n, 12]])
    list_dphi, list_dtheta, list_dpsi = derivation(list_phi, list_theta, list_psi, dt)
    list_d2phi, list_d2theta, list_d2psi = derivation(list_dphi, list_dtheta, list_dpsi, dt)
    list_d2x_cardan, list_d2y_cardan, list_d2z_cardan = derivation(list_dx_cardan, list_dy_cardan, list_dz_cardan, dt)
    
    ### les accélérations dans le repère de la fusée
    list_ax, list_ay, list_az = derivation(list_vx, list_vy, list_vz, dt)
    
    ### les accélérations mesurées dans la fusée
    list_agx, list_agy, list_agz = np.copy(list_ax), np.copy(list_ay), np.copy(list_az)
    # Ajout de l'accélération de pesanteur
    for i in range(len(Y)):
        latitude = gps.position_GPS(Y[i])[0]
        les_g_absolu = np.array([0, 0, -g_acceleration_pesanteur(latitude,list_z[0][i])])
        les_g_fusee = np.dot(Mat_fusee_terre(Y[i]), les_g_absolu)
        list_agx[0][i] += les_g_fusee[0]
        list_agy[0][i] += les_g_fusee[1]
        list_agz[0][i] += les_g_fusee[2]
    
    # Changement de point des accélération (du centre de gravité au point des accéléromètres)
    position_acc = [2.09,0.0,0.0]
    l_phi, l_theta, l_psi = angles_de_euler('zxz', list_q0, list_q1, list_q2, list_q3)
    l_dphi, l_dtheta, l_dpsi = derivation(l_phi, l_theta, l_psi, dt)
    ###l_d2phi, l_d2theta, l_d2psi = derivation(l_dphi, l_dtheta, l_dpsi, dt)

    # Calcul des vitesses angulaires sur les axes du repère des accélérations linéaires pour calculer les accélérations centrifuges
    omegax = l_dtheta*np.cos(l_phi)+l_dpsi*np.sin(l_theta)*np.sin(l_phi)
    omegay = -l_dtheta*np.sin(l_phi)+l_dpsi*np.sin(l_theta)*np.cos(l_phi)
    omegaz = l_dphi+l_dpsi*np.cos(l_theta)
    
    # Calcul des accélérations angulaires sur les axes du repère des accélérations linéaires pour calculer les accélérations dû aux accélérations angulaires
    domegax, domegay, domegaz = derivation(omegax, omegay, omegaz, dt)
    
    # Ajout des accélérations dû aux rotations aux accélérations mesurées 
    ##list_ax += (position_acc[0]-X_CG)*(omegay**2+omegaz**2)+position_acc[1]*domegaz-position_acc[2]*domegay
    ##list_ay += position_acc[1]*(omegax**2+omegaz**2)+position_acc[2]*domegax-position_acc[0]*domegaz
    ##list_az += position_acc[2]*(omegax**2+omegay**2)+position_acc[0]*domegay-position_acc[1]*domegax
    list_agx += (position_acc[0]-X_CG)*(omegay**2+omegaz**2)+position_acc[1]*domegaz-position_acc[2]*domegay
    list_agy += position_acc[1]*(omegax**2+omegaz**2)+position_acc[2]*domegax-position_acc[0]*domegaz
    list_agz += position_acc[2]*(omegax**2+omegay**2)+position_acc[0]*domegay-position_acc[1]*domegax
    

    
    ### Echantillonnage des données sur la période souhaitée
    
    list_x = echantillonnage(list_t, list_x, periode_ech)
    list_y = echantillonnage(list_t, list_y, periode_ech)  
    list_z = echantillonnage(list_t, list_z, periode_ech) 
    list_vx = echantillonnage(list_t, list_vx, periode_ech) 
    list_vy = echantillonnage(list_t, list_vy, periode_ech) 
    list_vz = echantillonnage(list_t, list_vz, periode_ech) 
    list_v = echantillonnage(list_t, list_v, periode_ech) 
    list_ax = echantillonnage(list_t, list_ax, periode_ech) 
    list_ay = echantillonnage(list_t, list_ay, periode_ech) 
    list_az = echantillonnage(list_t, list_az, periode_ech) 
    list_agx = echantillonnage(list_t, list_agx, periode_ech) 
    list_agy = echantillonnage(list_t, list_agy, periode_ech) 
    list_agz = echantillonnage(list_t, list_agz, periode_ech) 
    list_phi = echantillonnage(list_t, list_phi, periode_ech) 
    list_theta = echantillonnage(list_t, list_theta, periode_ech) 
    list_psi = echantillonnage(list_t, list_psi, periode_ech) 
    list_x_cardan = echantillonnage(list_t, list_x_cardan, periode_ech) 
    list_y_cardan = echantillonnage(list_t, list_y_cardan, periode_ech) 
    list_z_cardan = echantillonnage(list_t, list_z_cardan, periode_ech) 
    list_dx_cardan = echantillonnage(list_t, list_dx_cardan, periode_ech) 
    list_dy_cardan = echantillonnage(list_t, list_dy_cardan, periode_ech) 
    list_dz_cardan = echantillonnage(list_t, list_dz_cardan, periode_ech) 
    list_d2x_cardan = echantillonnage(list_t, list_d2x_cardan, periode_ech) 
    list_d2y_cardan = echantillonnage(list_t, list_d2y_cardan, periode_ech) 
    list_d2z_cardan = echantillonnage(list_t, list_d2z_cardan, periode_ech) 
    list_q0 = echantillonnage(list_t, list_q0, periode_ech) 
    list_q1 = echantillonnage(list_t, list_q1, periode_ech) 
    list_q2 = echantillonnage(list_t, list_q2, periode_ech) 
    list_q3 = echantillonnage(list_t, list_q3, periode_ech) 
    list_dphi = echantillonnage(list_t, list_dphi, periode_ech) 
    list_dtheta = echantillonnage(list_t, list_dtheta, periode_ech) 
    list_dpsi = echantillonnage(list_t, list_dpsi, periode_ech) 
    list_d2phi = echantillonnage(list_t, list_d2phi, periode_ech) 
    list_d2theta = echantillonnage(list_t, list_d2theta, periode_ech) 
    list_d2psi = echantillonnage(list_t, list_d2psi, periode_ech) 

    list_t = np.arange(list_t[0,0], list_t[0,-1], periode_ech)
    
    ### génération des données du milieu ambiant
    if not(magnetique is None) :
        list_B = generate_mag(list_x, list_y, list_z, gps, list_q0, list_q1, list_q2, list_q3)    
    if not(temperature is None) :
        list_T = generate_temperature(list_z)
    if not(pression is None) :
        list_P = generate_pression(list_z)
    
    ### position GPS
    list_GPS = generate_GPS(list_x, list_y, list_z, gps)
    
    ##### Redaction du fichier de résultat
    separateur = '\t' # separateur utiliser par le tableur

    ### Ouverture du fichier
    f = open('resultats/resultats_'+name+'.csv','w')
    
    ### Ecriture des titres
    if not(temps is None) :
        f.write('Temps'+separateur)
    if not(x is None) :
        f.write('Position en x'+separateur)
    if not(y is None) :
        f.write('Position en y'+separateur)
    if not(z is None) :
        f.write('Position en z'+separateur)
    if not(vitesses is None) :
        f.write('Vitesse en x'+separateur)
        f.write('Vitesse en y'+separateur)
        f.write('Vitesse en z'+separateur)
    if not(norme_vitesse is None) :
        f.write('Vitesse globale'+separateur)
    if not(accelerations is None) :
        f.write('Accélération en x'+separateur)
        f.write('Accélération en y'+separateur)
        f.write('Accélération en z'+separateur)
    if not(accelerations_mesurées is None) :    
        f.write('Accélération en x avec g'+separateur)
        f.write('Accélération en y avec g'+separateur)
        f.write('Accélération en z avec g'+separateur)
    if not(angles_euler is None) :
        f.write('Angle phi'+separateur)
        f.write('Angle theta'+separateur)
        f.write('Angle psi'+separateur)
    if not(angles_cardan is None) :
        f.write('Angle x'+separateur)
        f.write('Angle y'+separateur)
        f.write('Angle z'+separateur)
        f.write('Vitesse angulaire dx'+separateur)
        f.write('Vitesse angulaire dy'+separateur)
        f.write('Vitesse angulaire dz'+separateur)
        f.write('Accélération angulaire d2x'+separateur)
        f.write('Accélération angulaire d2y'+separateur)
        f.write('Accélération angulaire d2z'+separateur)
    if not(quaternions is None) :
        f.write('Quaternion q0'+separateur)
        f.write('Quaternion q1'+separateur)
        f.write('Quaternion q2'+separateur)
        f.write('Quaternion q3'+separateur)
    if not(vitesses_angulaires is None) :
        f.write('Vitesse angulaire dphi'+separateur)
        f.write('Vitesse angulaire dptheta'+separateur)
        f.write('Vitesse angulaire dpsi'+separateur)
    if not(accelerations_angulaires is None) :
        f.write('Accélération angulaire d2phi'+separateur)
        f.write('Accélération angulaire d2ptheta'+separateur)
        f.write('Accélération angulaire d2psi'+separateur)
    if not(magnetique is None) :
        f.write('F champ magnétique (en nT)'+separateur)
        f.write('H champ magnétique (en nT)'+separateur)
        f.write('X champ magnétique (en nT)'+separateur)
        f.write('Y champ magnétique (en nT)'+separateur)
        f.write('Z champ magnétique (en nT)'+separateur)
        f.write('Déclinaison du champ magnétique (en deg)'+separateur)
        f.write('Inclinaison du champ magnétique (en deg)'+separateur)
    if not(temperature is None) :
        f.write('Température ambiante (en K)'+separateur)
    if not(pression is None) :
        f.write('Pression ambiante (en Pa)'+separateur)
    if not(GPS is None) :
        f.write('Latitude (en °)'+separateur)
        f.write('Longitude (en °)'+separateur)
        f.write('Altitude (en m)'+separateur)
    f.write('\n')

    ### Ecriture des données
    for i in range(len(list_t)):
        if not(temps is None):
            f.write(str(list_t[i]).replace('.',',')+separateur)
        if not(x is None):
            f.write(str(list_x[i]).replace('.',',')+separateur)
        if not(y is None):
            f.write(str(list_y[i]).replace('.',',')+separateur)
        if not(z is None):
            f.write(str(list_z[i]).replace('.',',')+separateur)
        if not(vitesses is None):
            f.write(str(list_vy[i]).replace('.',',')+separateur)
            f.write(str(list_vz[i]).replace('.',',')+separateur)
            f.write(str(list_vx[i]).replace('.',',')+separateur)
        if not(norme_vitesse is None):
            f.write(str(list_v[i]).replace('.',',')+separateur)
        if not(accelerations is None):
            f.write(str(-list_ay[i]).replace('.',',')+separateur)
            f.write(str(list_az[i]).replace('.',',')+separateur)
            f.write(str(list_ax[i]).replace('.',',')+separateur)
        if not(accelerations_mesurées is None) :   
            f.write(str(-list_agy[i]).replace('.',',')+separateur)
            f.write(str(list_agz[i]).replace('.',',')+separateur)
            f.write(str(list_agx[i]).replace('.',',')+separateur)
        if not(angles_euler is None):
            f.write(str(list_phi[i]).replace('.',',')+separateur)
            f.write(str(list_theta[i]).replace('.',',')+separateur)
            f.write(str(list_psi[i]).replace('.',',')+separateur)
        if not(angles_cardan is None):
            f.write(str(list_x_cardan[i]).replace('.',',')+separateur)
            f.write(str(list_y_cardan[i]).replace('.',',')+separateur)
            f.write(str(list_z_cardan[i]).replace('.',',')+separateur)
            f.write(str(list_dx_cardan[i]).replace('.',',')+separateur)
            f.write(str(list_dy_cardan[i]).replace('.',',')+separateur)
            f.write(str(list_dz_cardan[i]).replace('.',',')+separateur)
            f.write(str(list_d2x_cardan[i]).replace('.',',')+separateur)
            f.write(str(list_d2y_cardan[i]).replace('.',',')+separateur)
            f.write(str(list_d2z_cardan[i]).replace('.',',')+separateur)
        if not(quaternions is None):
            f.write(str(list_q0[i]).replace('.',',')+separateur)
            f.write(str(list_q1[i]).replace('.',',')+separateur)
            f.write(str(list_q2[i]).replace('.',',')+separateur)
            f.write(str(list_q3[i]).replace('.',',')+separateur)
        if not(vitesses_angulaires is None):
            f.write(str(list_dphi[i]).replace('.',',')+separateur)
            f.write(str(list_dtheta[i]).replace('.',',')+separateur)
            f.write(str(list_dpsi[i]).replace('.',',')+separateur)
        if not(accelerations_angulaires is None):
            f.write(str(list_d2phi[i]).replace('.',',')+separateur)
            f.write(str(list_d2theta[i]).replace('.',',')+separateur)
            f.write(str(list_d2psi[i]).replace('.',',')+separateur)
        if not(magnetique is None):
            for j in range(len(list_B[i])):
                f.write(str(list_B[i][j]).replace('.',',')+separateur)
        if not(temperature is None):
            f.write(str(list_T[i]).replace('.',',')+separateur)
        if not(pression is None):
            f.write(str(list_P[i]).replace('.',',')+separateur)
        if not(GPS is None):
            for j in range(len(list_GPS[i])):
                f.write(str(list_GPS[i][j]).replace('.',',')+separateur)
        f.write('\n')

    ### Fermeture du fichier
    f.close()

def derivation(list_vx, list_vy, list_vz, dt):
    """
    Paramètres : 
    - list_vx, list_vy, list_vz : les listes des variables à dériver dans les trois dimensions
    - dt : le pas de temps
    
    Retourne : 3 tableaux contenant les listes de valeurs des dérivées sur les 3 axes

    La dérivation est fait le calcul du taux d'accroissement entre de valeurs succéssives.
    /!\ En sortie ce sont des matrices colonnes /!\
    """

    list_ax, list_ay, list_az = [0], [0], [0]
    for i in range(len(list_vx[0])-1):
        list_ax.append((list_vx[0,i+1]-list_vx[0,i])/dt)
        list_ay.append((list_vy[0,i+1]-list_vy[0,i])/dt)
        list_az.append((list_vz[0,i+1]-list_vz[0,i])/dt)
    return np.array([list_ax]), np.array([list_ay]), np.array([list_az])

def echantillonnage(list_t, liste, periode_ech):
    """
    Paramètres : 
    - list_t : la matrice colonne des temps non échantillonnée
    - liste : la matrice colonne à échantillonner
    - periode_ech : la période d'échantillonnage

    Utilisation d'une interpolation linéaire

    Retourne la liste échantillonnée avec la période souhaitée

    /!\ En entrée ce sont des matrices colonnes /!\
    """
    ech_t = np.arange(list_t[0,0], list_t[0,-1], periode_ech)

    array = np.concatenate((list_t,liste),axis = 0).T
    liste = []
    for t in ech_t:
        liste.append(interpolation(array,t))
    return liste

def angles_de_euler(sequence, list_q0, list_q1, list_q2, list_q3):
    """
    Paramètres : 
    - sequence : la séquence de rotation des angles d'euler souhaités
    - list_q0, list_q1, list_q2, list_q3 : listes des quaternions de la fusée au cours du vol

    Retourne les angles d'Euler dans 3 matrices colonnes
    """
    list_phi = []
    list_theta = []
    list_psi = []

    for i in range(len(list_q0[0])):
        euler = quat2eul(sequence, [list_q0[0][i], list_q1[0][i], list_q2[0][i], list_q3[0][i]])
        list_phi.append(euler[0])
        list_theta.append(euler[1])
        list_psi.append(euler[2])
    return np.array([list_phi]), np.array([list_theta]), np.array([list_psi])

def generate_mag (list_x, list_y, list_z, gps, list_q0, list_q1, list_q2, list_q3):
    """
    Paramètres :
    - list_x, list_y, list_z : la liste des positions au cours du temps
    - gps : l'object permettant de calculer la position gps de la fusée
    
    Retourne : le tableau des grandeures du champs
    """
    list_B = []
    for i in range(len(list_x)):
        Xi = [list_x[i], list_y[i], list_z[i]]
        Yi = [0,0,0,0,0,0,list_q0[i],list_q1[i],list_q2[i],list_q3[i],0,0,0]
        lat, lon, alt = gps.position_GPS(Xi)
        if sys.platform.startswith('win32'):
            addrcodec = '.\WMM2020\src\MagnetoCompiledWindows.exe'
        elif sys.platform.startswith('linux'):
            addrcodec = './WMM2020/src/MagnetoCompiledLinux'
        elif sys.platform.startswith('darwin'):
            print('Le programme n a pas été testé sous Mac Os\n Il peut y avoir des problèmes avec le magnétomètre\n')
        else :
            print('Os non reconnu\n')
        latitude = str(lat)
        longitude = str(lon)
        altitude = str(alt/1000)
        date = '2020'
        commande = addrcodec+' '+latitude+' '+longitude+' '+altitude+' '+date
        p1 = subprocess.run(commande,shell=True, capture_output=True, text=True)
        mesmag = p1.stdout
        listemag = mesmag.split(' ')
        B_mesures = []
        for mes in listemag:
            if mes!='':
                B_mesures.append(mes.strip('\t').strip('\n'))
        B_mesures = np.array(B_mesures)
        Bx, By, Bz = B_mesures[2],B_mesures[3],B_mesures[4]
        B  = np.array([Bx, By, Bz])
        B_xyz = np.dot(Mat_fusee_terre(Yi), B)
        B_mesures[2] = B_xyz[0]
        B_mesures[3] = B_xyz[1]
        B_mesures[4] = B_xyz[2]
        list_B.append(B_mesures)
    return (np.array(list_B))    

def generate_temperature(list_z):
    """
    Paramètres :
    - list_z : la liste des altitudes au cours du vol

    Retourne la liste des températures ambiantes autour de la fusée au cours du vol
    """
    list_T = []
    for z in list_z:
        list_T.append(Temperature(z))
    return np.array(list_T)

def generate_pression(list_z):
    """
    Paramètres : 
    - list_z : la liste des altitudes au cours du vol

    Retourne la liste des pressions ambiantes autour de la fusée au cours du vol
    """
    list_P = []
    for z in list_z:
        list_P.append(Pression(z, Temperature(z)))
    return np.array(list_P)

def generate_GPS(list_x, list_y, list_z, gps):
    """
    Paramètres : 
    - list_x, list_y, list_z : la liste des positions au cours du temps
    - gps : l'object permettant de calculer la position gps de la fusée
    
    Retourne : le tableau des positions GPS
    """
    list_GPS = []
    for i in range(len(list_x)):
        X = [list_x[i], list_y[i], -list_z[i]]
        lat, lon, alt = gps.position_GPS(X)
        list_GPS.append([lat, lon, alt])
    return np.array(list_GPS)




def result_sensors (name,my_rocket, t):
    """
    Paramètres : my_rocket la variable contenant la fusée simulée, t la liste des temps
    Produit un tableau au format .csv avec l'ensemble des données mesurées par les capteurs de la fusée
    Ce fichier est nommé "sensors_data.csv" et se trouve dans le dossier "resultats" 
    /!\ n'oubliez pas de fermer le fichier avant de relancer la simulation, sinon il ne sera pas mis à jour /!\
    """
    
    separateur = '\t' # Définission du séparateur pour le tableur

    ### Ouverture du fichier 
    f = open('resultats/sensors_data_'+name+'.csv','w')
    
    ### Rédaction des titres
    f.write('Temps'+separateur)
    i = 1
    for capteur in my_rocket.fusee.accelerometres_list:
        for axe in ['x (en m/s²)', 'y (en m/s²)', 'z (en m/s²)']:
            f.write('Accelerometre '+str(i)+' '+axe+separateur)
        i+=1
    i = 1
    for capteur in my_rocket.fusee.gyroscopes_list:
        for angle in ['phi', 'theta', 'psi']:
            f.write('Gyroscope '+str(i)+' '+angle+separateur)
        i+=1
    i = 1
    for capteur in my_rocket.fusee.gyrometres_list:
        for angle in ['phi', 'theta', 'psi']:
            f.write('Gyrometre '+str(i)+' '+angle+separateur)
        i+=1
    i = 1
    for capteur in my_rocket.fusee.barometres_list:
        f.write('Barometre '+str(i)+separateur)
        i+=1
    i = 1
    for capteur in my_rocket.fusee.thermometres_list:
        f.write('Thermometre '+str(i)+separateur)
        i+=1
    i = 1   
    for capteur in my_rocket.fusee.magnetometres_list:
        if len(capteur.liste_mesures[i])==3:
            for valeur in ['X (en nT)','Y (en nT)','Z (en nT)']:
                f.write('Magnetometre '+str(i)+' '+valeur+separateur)
        else :
            for valeur in ['F (en nT)','H (en nT)','X (en nT)','Y (en nT)','Z (en nT)','Declinaison (en deg)','Inclinaison (en deg)']:
                f.write('Magnetometre '+str(i)+' '+valeur+separateur)
        i+=1
    i = 1
    for capteur in my_rocket.fusee.GPS_list:
        for direction in ['Latitude (en deg)', 'Longitude (en deg)', 'Altitude (en deg)']:
            f.write('GPS '+str(i)+' '+direction+separateur)
    
    f.write('\n') 


    ### Rédactions des données

    for i in range(len(t)):
        f.write(str(t[i]).replace('.',',')+separateur)
        for capteur in my_rocket.fusee.accelerometres_list:
            for axe in range(len(capteur.liste_mesures[i])):
                f.write(str(capteur.liste_mesures[i][axe]).replace('.',',')+separateur)
        for capteur in my_rocket.fusee.gyroscopes_list:
            for angle in range(len(capteur.liste_mesures[i])):
                f.write(str(capteur.liste_mesures[i][angle]).replace('.',',')+separateur)
        for capteur in my_rocket.fusee.gyrometres_list:
            for angle in range(len(capteur.liste_mesures[i])):
                f.write(str(capteur.liste_mesures[i][angle]).replace('.',',')+separateur)
        for capteur in my_rocket.fusee.barometres_list:
            f.write(str(capteur.liste_mesures[i]).replace('.',',')+separateur)
        for capteur in my_rocket.fusee.thermometres_list:
            f.write(str(capteur.liste_mesures[i]).replace('.',',')+separateur)
        for capteur in my_rocket.fusee.magnetometres_list:
            for valeur in range(len(capteur.liste_mesures[i])):
                f.write(str(capteur.liste_mesures[i][valeur]).replace('.',',')+separateur)
        for capteur in my_rocket.fusee.GPS_list:
            for valeur in range(len(capteur.liste_mesures[i])):
                f.write(str(capteur.liste_mesures[i][valeur]).replace('.',',')+separateur)
        f.write('\n')
    
    ### Fermeture du fichier
    f.close()



