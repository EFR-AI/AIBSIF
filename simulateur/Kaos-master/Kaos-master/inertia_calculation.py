from numpy import cos, sin, tan, arccos, arctan, arcsin
import numpy as np




def mat_fusee_tuyere(psi,theta):
    """
    paramètre :angle de la tuyère psi et theta
    Matrice de changement de base entre le repère de la fusée et le repere de la tuyère
    """
    return np.array([[cos(psi)*cos(theta)             , sin(psi)*cos(theta)              , -sin(theta)        ],
                     [-sin(psi)                       , cos(psi)                         , 0                  ],
                     [+cos(psi)*sin(theta)            , sin(psi)*sin(theta)              , cos(theta)         ]])


def matrice_inertie_fusée(my_rocket, psi,theta):
    """
    paramètres : psi et theta les angles des la tuyères tels que définis sur le document de référence
    retourne la matrice d'inertie de la fusée
    """
    #On récupère les matrices d'inerties des difféfrents éléments au centre de gravité de la fusée:
    inertie_moteur = my_rocket.inertie.inertie_moteur
    matrice_inertie_ogive = my_rocket.inertie.inertie_ogive
    matrice_inertie_corps = my_rocket.inertie.inertie_corps
    matrice_inertie_des_ailerons = my_rocket.inertie.inertie_ailerons
        
        
    #On exprime la matrice d'inertie de la tuyère en fonction de ses angles au centre de gravité de la fusée
    mat_pas_tuyere = mat_fusee_tuyere(psi,theta)
    mat_pas_tuyere_inv = np.linalg.inv(mat_pas_tuyere)
    matrice_inertie_tuyere_angle = np.dot( np.dot(mat_pas_tuyere_inv,inertie_moteur),mat_pas_tuyere)
    X_cg = my_rocket.fusee.X_CG - my_rocket.moteur.X
    matrice_inertie_tuyere_angle += my_rocket.moteur.masse*np.array([[0,0,0],
                                                                        [0,X_cg**2,0],
                                                                        [0,0,X_cg**2]])

    #On détermine la matrice d'inertie de la fusée au niveau du centre de gravité
    matrice_inertie = matrice_inertie_ogive + matrice_inertie_des_ailerons + matrice_inertie_corps + matrice_inertie_tuyere_angle 
    return matrice_inertie
        
        

