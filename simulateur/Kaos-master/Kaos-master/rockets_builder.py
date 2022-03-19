
import subprocess, sys
from typing import Sequence
import numpy as np
from numpy import cos, sin, random
#from numpy.lib.function_base import _select_dispatcher
from g_variable import g_acceleration_pesanteur
from quaternion import Mat_fusee_terre, quat2eul
from math import sqrt
from thermodynamic_model import Temperature, Pression
#________________________________________________________#
#                                                        #
#/!\ Les longueurs sont ici en m et les masses en kg /!\#
#________________________________________________________#

###############################################################################################################################################################################

class Aileron:
    """
    Classe des Ailerons
    e,n,p,m sont des paramètres géométrique de l'aileron, se réferer au README
    d_Aileron : le diamètre du corps au niveau de l'aileron
    X_Aileron : distance au sommet de la fusée
    masse : masse d'UN aileron
    nb_ailerons : le nombre d'ailerons

    """
    def __init__(self,e,n,p,m,d_Aileron,X_Aileron, masse, nb_ailerons, inertie = None): #Se referer au document vol-de-la-fusee.pdf dans le drive /simulation/etat de l'art  à partir de la page 19
        
        #Paramètres dimensionnant de l'Aileron
        #_________________________
        self.e = e
        self.n = n
        self.p = p
        self.m = m
        self.d_Aileron = d_Aileron
        self.X_Aileron = X_Aileron
        self.masse = masse
        self.nb_ailerons = nb_ailerons

        if inertie is None:
            self.inertie = None   
        else:
            self.inertie = inertie
        #_________________________
        
        
    @property
    def f(self): #Simplification d'expression
        return np.sqrt(self.e**2+(self.p+(self.n-self.m)/2)**2)
    
    @property
    def Cn_alpha(self): #Coefficient de portance des ailerons
        """ 
        Coefficient de portance des ailerons
        """
        return (self.nb_ailerons*(1 + (self.d_Aileron/(2*self.e + self.d_Aileron))) * 4 * (self.e/self.d_Aileron)**2)/(1+np.sqrt(1 + (2*self.f/(self.m+self.n))**2))
    
    @property    
    def X_CPA(self): #Distance X du centre de portance aérodynamique induit par l'Aileron, à la pointe de l'Ogive
        """
        Distance X du centre de portance aérodynamique induit par l'Aileron, à la pointe de l'Ogive
        """
        return self.X_Aileron + self.p * (self.m+2*self.n) / (3*(self.m+self.n)) + (1/6)*(self.m+self.n-(self.m*self.n/(self.m+self.n)))
    
    @property    
    def X_CG(self): #Distance X du centre de gravité
        """
        Distance X du centre de gravité
        """
        return self.X_Aileron
    
###############################################################################################################################################################################
    
class Ogive:
    """
    Classe des Ogives
    
    Prend en argument les dimensions de l'Ogive et la forme voulue (parabolique, ogivale, elliptique, conique).
    
    Par défaut l'Ogive sera parabolique
    
    H: Hauteur de l'Ogive en m 
    
    d_Ogive : Diamètre de la base de l'Ogive en m
    
    d_ref : d_Ogive en m 
    
    """
    def __init__(self,H,d_ogive, masse, forme = "parabolique", inertie = None):
        
        #Paramètres dimensionnant de l'Ogive
        #_________________________
        self.H = H
        self.d_ogive = d_ogive
        self.d_ref = d_ogive
        self.forme = forme	
        self.masse = masse

        if inertie is None:
            self.inertie = None   
        else:
            self.inertie = inertie

        #_________________________
        
    @property
    def X_CPA(self): #Distance X du centre de portance aérodynamique induit par l'Ogive, à la pointe de l'Ogive
        """
        Distance X du centre de portance aérodynamique induit par l'Ogive, à la pointe de l'Ogive
        """
        if self.forme == "parabolique":
            return (1/2) * self.H
        if self.forme == "ogivale":
            return (7/15) * self.H
        if self.forme == "elliptique":
            return (1/3) * self.H
        if self.forme == "conique":
            return (2/3) * self.H
        
    @property
    def Cn_alpha(self): #Coefficient de portance de l'Ogive.
        """
        Coefficient de portance de l'Ogive
        """
        return 2 * (self.d_ogive/self.d_ref)**2   

    @property    
    def X_CG(self): #Distance X du centre de gravité
        """
        Distance X du centre de gravité
        """
        return self.H/2    

####################################################################################################################################################################

class Corps:
    """
    Classe des corps
    
    Prend en argument les dimensions du corps.
    
    H: Hauteur du corps en m
    
    d_corps : Diamètre du corps en m
    
    masse : masse du reservoir à vide en kg

    masse_ergol : masse de carburant dans le reservoir plein en kg

    X : La distance au sommet en m
    
    """
    def __init__(self, H, d_corps, masse, masse_ergol, X, inertie = None):
        
        #Paramètres dimensionnant de l'Ogive
        #_________________________
        self.H = H
        self.d_corps = d_corps
        self.masse = masse  #à vide
        self.masse_ergol = masse_ergol
        self.X = X

        if inertie is None:
            self.inertie = None   
        else:
            self.inertie = inertie
        #_________________________
        
        

    @property    
    def X_CG(self): #Distance X du centre de gravité
        """
        Distance X du centre de gravité
        """
        return self.H/2+self.X 
    
################################################################################################################################################################################

class Jupe_retreint:
    """
    Classe des Jupes et des rétreints
    
    Prend en argument les dimensions de la Jupe/retreint:
        
        H : Hauteur de la Jupe/retreint en m 
        
        X : Distance au sommet de l'Ogive en m
        
        d1: Diamètre supérieur en m 
        
        d2: Diamètre inférieur en m 
        
        masse : Masse de l'ogive en kg
    """
    def __init__(self, H, X, d1, d2, masse):
        
        #Paramètres dimensionnant de la jupe/ du retreint
        #_________________________
        self.H = H
        self.d1 = d1
        self.d2 = d2
        self.X = X
        self.masse = masse
        #_________________________
        
    @property
    def X_CPA(self): #Distance X du centre de portance aérodynamique induit par la jupe / le retreint, à la pointe de l'Ogive
        """
        Distance X du centre de portance aérodynamique induit par la jupe / le retreint, à la pointe de l'Ogive
        """
        return self.X + (self.H/3)*(1+(1/(1+(self.d1/self.d2))))
        
    @property
    def Cn_alpha(self): #Coefficient de portance du retreint / de la jupe.
        """
        Coefficient de portance du retreint / de la jupe
        """
        return 2 * ((self.d2/self.d1)**2 - (self.d1/self.d1)**2) #rechercher la formule, la formule utilisée a été modifié au hasard
    
    @property    
    def X_CG(self): #Distance X du centre de gravité
        """
        Distance X du centre de gravité
        """
        return self.H/2 + self.X
    
################################################################################################################################################################################
        
class Parachute :

    """
    Classe des parachutes. Prend en argument les paramètres physiques du parachutes
    
    S : Surface du parachute en m^2
    
    masse : Masse en kg 

    X : La distance au sommet en m

    alt : Altitude de déploiement en m
    """
    
    def __init__(self, surface, masse, X, alt = None, apogee = False):
        
        self.surface = surface
        self.masse = masse
        self.apogee = apogee
        self.X = X
        
        if alt is None:
            self.alt = None
        else:
            self.alt = alt

################################################################################################################################################################################
        
class Moteur :

    """
    Classe des moteurs. Prend en argument les paramètres physiques du moteur
    
    thrust : Tableaux de couples temps-puissance
    
    masse : Masse
    
    h : hauteur de la tuyère
    
    R : rayon de la tuyère
    
    X : position du moteur
    
    """

    def __init__(self, thrust, masse, h, R, X, inertie = None):
        
        self.thrust = thrust
        self.masse = masse
        self.h = h
        self.R = R
        self.X = X
        if inertie is None:
            self.inertie = None   
        else:
            self.inertie = inertie

################################################################################################################################################################################                       

class RCS : 
    """
    Classe des RCS. Prend en argument les paramètres physiques d'un RCS
    
    axis : Direction de poussée du RCS (Au choix y, -y, z, -z) 
    
    X : Position du RCS le long de la fusée par rapport au sommet de la fusée  en m 
    
    thrust : Poussée du RCS en N
    
    masse : Masse en kg 

    We'll assume y is theta = 0

    """
    def __init__(self, axis, X, thrust, masse):

        self.axis = axis
        self.X = X
        self.thrust = thrust
        self.masse = masse

################################################################################################################################################################################

class tuyere:
    """
    Classe des tuyères
    
    Prend en argument les dimension de la tuyère
    
    h : hauteur de la tuyère
    
    R : rayon de la tuyère
    
    masse : masse de la tuyère
    
    """
    def __init__(self,h,R,masse):
        #Paramètres dimensionnant de la tuyère
        #_________________________
        self.h = h
        self.R = R
        self.masse = masse  
        #_________________________
        
################################################################################################################################################################################                

class Fusee:
    
    """
    Classe de la fusée. Prend en argument les paramètres physiques/géométrique de la fusée ainsi que ses différentes pièces constituantes)
    
    aux_masse : Masse qui ne proviennent pas explicitement d'une partie (masse inerte, anneau de serrage...)
    
    corps_list : Liste d'instance de Corps de la classe Corps

    moteur : Instance de moteur de la classe Moteur
           
    aileron_list : Liste d'instance d'Aileron de la classe Aileron
    
    ogive : Instance d'Ogive de la classe Ogive
    
    jupe_retreint_list : Liste d'instance de Jupe et retreint de la classe Jupe_retreint

    parachutes_list : Liste d'instance de Parachute de la classe Parachute

    rcs_list : Liste d'instance de RCS de la classe RCS

    cn, cpa, cg , o  = Cnalpha, centre de portance, centre de gravité, point de ref aero 

    Coeff : Coeff aero dynamique, dans l'ordre Cl0, Clp, Cmq, Cnr
    """
    
    def __init__(self, aux_masse, corps_list, moteur, aileron_list = None, ogive = None , jupe_retreint_list = None, parachutes_list = None, accelerometres_list = None, gyroscopes_list = None, gyrometres_list = None, barometres_list = None, magnetometres_list = None, thermometres_list = None, GPS_list = None, rcs_list = None, cn = None, cg = None, cpa = None, o = None, Coeff = [0,0,0,0]):
        
        #Paramètres dimensionnant de la fusée
        #_________________________  
        self.moteur = moteur
        self.aux_masse = aux_masse
        self.cn = cn
        self.cg = cg
        self.cpa = cpa
        self.o = o
        self.Coeff = Coeff
        
        if ogive is None:
            self.ogive = Ogive(50, 25.3, 0.967) 
        else: 
            self.ogive = ogive

        if corps_list is None:
            self.corps_list = []
        else:
            self.corps_list = corps_list

        if aileron_list is None:
            self.aileron_list = []   #Aileron(30, 9, 21, 35, 25.3, 565, 0.599, 4)
        else: 
            self.aileron_list = aileron_list

        if jupe_retreint_list is None:
            self.jupe_retreint_list = []   
        else:
            self.jupe_retreint_list = jupe_retreint_list

        if parachutes_list is None:
            self.parachutes_list = []   
        else:
            self.parachutes_list = parachutes_list

        if rcs_list is None:
            self.rcs_list = []   
        else:
            self.rcs_list = rcs_list

        if accelerometres_list is None:
            self.accelerometres_list = []
        else:
            self.accelerometres_list = accelerometres_list

        if gyroscopes_list is None:
            self.gyroscopes_list = []
        else:
            self.gyroscopes_list = gyroscopes_list

        if gyrometres_list is None:
            self.gyrometres_list = []
        else:
            self.gyrometres_list = gyrometres_list

        if barometres_list is None:
            self.barometres_list = []
        else:
            self.barometres_list = barometres_list

        if magnetometres_list is None:
            self.magnetometres_list = []
        else:
            self.magnetometres_list = magnetometres_list

        if thermometres_list is None:
            self.thermometres_list = []
        else:
            self.thermometres_list = thermometres_list
        
        if GPS_list is None:
            self.GPS_list = []
        else:
            self.GPS_list = GPS_list
        
        #_________________________
    
    @property
    def masse_vide(self):
        """
        Donne la masse à vide de la fusée
        """
        m_para = 0
        m_rcs = 0
        m_corps = 0
        m_ailerons = 0
        m_jr = 0

        for objet in self.parachutes_list:
            m_para += objet.masse

        for objet in self.rcs_list:
            m_rcs += objet.masse

        for objet in self.corps_list:
            m_corps += objet.masse
            
        for objet in self.aileron_list:
            m_ailerons += objet.masse*objet.nb_ailerons
        
        for objet in self.jupe_retreint_list:
            m_jr += objet.masse

        return m_corps + m_ailerons + self.ogive.masse + self.moteur.masse + self.aux_masse + m_para + m_rcs + m_jr

    @property
    def masse_ergol(self):
        """
        Donne la masse de carburant de la fusée
        """
        masse_ergol = 0
        for objet in self.corps_list:
            masse_ergol += objet.masse_ergol
        return masse_ergol
    
    @property
    def masse(self):
        """
        Donne la masse de la fusée avec les ergols
        """
        return self.masse_vide + self.masse_ergol

    @property
    def hauteur(self):
        """
        Calcule la hauteur de la fusée en sommant la hauteur de l'ogive, les corps et les jupes/retreint
        """
        H_jr = 0
        H_corps = 0

        for objet in self.jupe_retreint_list:
            H_jr += objet.H

        for objet in self.corps_list:
            H_corps += objet.H

        return self.ogive.H + H_corps + H_jr

    @property
    def Cn_alpha(self): #Coefficient de portance de la fusée
        """
        Coefficient de portance de la fusée
        """
        if self.cn != None:
            return self.cn
        
        else:
            Cn_alpha_Jupe_retreint = 0
            Cn_alpha_Ailerons = 0
        
            for objet in self.jupe_retreint_list:
                Cn_alpha_Jupe_retreint += objet.Cn_alpha
            
            for objet in self.aileron_list:
                Cn_alpha_Ailerons += objet.Cn_alpha
            
            return Cn_alpha_Ailerons + self.ogive.Cn_alpha + Cn_alpha_Jupe_retreint
    
    @property
    def X_CPA(self): #Distance X du centre de portance aérodynamique de la fusée, à la pointe de l'Ogive
        """
        Distance X du centre de portance aérodynamique de la fusée, à la pointe de l'Ogive
        """
        if self.cpa != None: 
            return self.cpa

        else:
            XC = 0
        
            for objet in self.jupe_retreint_list: 
                XC += objet.X_CPA * objet.Cn_alpha
            
            for objet in self.aileron_list:
                XC += objet.X_CPA * objet.Cn_alpha
        
            return (self.ogive.X_CPA * self.ogive.Cn_alpha + XC) / self.Cn_alpha

    @property
    def X_CG(self):
        """
        Distance X du centre de gravitée de la fusée, à la pointe de l'Ogive
        """
        if self.cg != None:
            return self.cg

        else:
            XCG_para = 0
            XCG_rcs = 0
            XCG_corps = 0
            XCG_ailerons = 0
            XCG_jr = 0

            for objet in self.parachutes_list:
                XCG_para += objet.masse*objet.X

            for objet in self.rcs_list:
                XCG_rcs += objet.masse*objet.X

            for objet in self.corps_list:
                XCG_corps += (objet.masse + objet.masse_ergol)*objet.X
            
            for objet in self.aileron_list:
                XCG_ailerons += objet.masse*objet.nb_ailerons*objet.X_Aileron
        
            for objet in self.jupe_retreint_list:
                XCG_jr += objet.masse*objet.X

        
            return (XCG_ailerons + self.ogive.masse*self.ogive.X_CG + XCG_jr + XCG_corps + XCG_para + XCG_rcs + self.moteur.X*self.moteur.masse) / (self.masse - self.aux_masse)
    
    @property
    def X_O(self):

        if self.o != None:
            return self.o

        else:
            return self.X_CG
      
################################################################################################################################################################################  

class inertie :
    """
    Classe des inerties. Prends en argument les différents constituants de la fusée avec leurs paramètres geométrique.
    
    Aileron : Instance d'Aileron de la classe Aileron
    
    Ogive : Instance d'Ogive de la classe Ogive
    
    Calcule les matrices d'inerties des constituants.
    
    """
    
    def __init__ (self, rocket):
        #____________________________
        self.corps_list = rocket.corps_list
        self.ogive = rocket.ogive
        self.aileron_list = rocket.aileron_list
        self.moteur = rocket.moteur
        self.rocketX_CG = rocket.X_CG
        #____________________________
        
    @property
    def inertie_corps(self):
        """
        Matrice d'inertie des corps au centre de gravité de la fusée 
        """
        I_CG_corps = np.array([[0.,0.,0.],
                              [0.,0.,0.],
                              [0.,0.,0.]])
        for objet in self.corps_list:
            mL2 = (objet.masse+objet.masse_ergol)*(self.rocketX_CG - objet.X_CG)**2
            L_CG = np.array([[0.,0.,0.],[0.,mL2,0.],[0.,0.,mL2]])
            if objet.inertie != None:
                I_CG_corps += objet.inertie + L_CG

            else:
                R_corps = objet.d_corps/2
                Ix, Iy = (objet.masse+objet.masse_ergol)*R_corps**2/2, (objet.masse+objet.masse_ergol)*(R_corps**2/4+objet.H**2/12)
                I_CG_corps += np.array([[Ix,0.,0.],[0.,Iy,0.],[0.,0.,Iy]]) + L_CG
        return I_CG_corps
    
    @property
    def inertie_moteur(self):
        """
        Matrice d'inertie du moteur en son centre de gravité
        """
        if self.moteur.inertie != None:
            return self.moteur.inertie

        else:
            return np.array([[self.moteur.masse*self.moteur.R**2/2        , 0.                                                      , 0.        ],
                            [0.                             , self.moteur.masse*self.moteur.R**2/4 + self.moteur.masse*self.moteur.h**2/12        ,0.      ],
                            [0.                             , 0.                                                      , self.moteur.masse*self.moteur.R**2/4 + self.moteur.masse*self.moteur.h**2/12          ]])
    
    @property
    def inertie_ogive(self):
        """
        Matrice d'inertie de l'ogive au centre de gravité de la fusée 
        """
        mL2 = self.ogive.masse*(self.rocketX_CG - self.ogive.X_CG)**2
        L_CG = np.array([[0.,0.,0.],[0.,mL2,0.],[0.,0.,mL2]])
        if self.ogive.inertie != None:
            return self.ogive.inertie + L_CG
        
        else:
            R = self.ogive.d_ogive
            #Calcul des coefficients
            Ix = self.ogive.masse*R**2/2
            Iy = self.ogive.masse*R**2/4 + self.ogive.masse*self.ogive.H**2/12
            return np.array ([[Ix,0.,0.],
                              [0.,Iy,0.],
                              [0.,0.,Iy]]) + L_CG
    
    @property
    def inertie_ailerons(self):
        """
        Matrice d'inertie des ailerons au centre de gravité de la fusée 
        """
        I_CG_ailerons = np.array([[0.,0.,0.],
                                  [0.,0.,0.],
                                  [0.,0.,0.]])
        for objet in self.aileron_list:
            mL2 = (objet.masse)*(self.rocketX_CG - objet.X_CG)**2
            L_CG = np.array([[0.,0.,0.],[0.,mL2,0.],[0.,0.,mL2]])
            if objet.inertie != None:
                I_CG_ailerons += objet.inertie + L_CG
            else:
                ### Matrice inertie d'un aileron

                #Défintion des paramètres

                Xa = objet.X_CPA
                Ma = objet.masse
                m, p, n, e = objet.m, objet.p, objet.n, objet.e
                m_s_a = Ma/((p*e/2)+e*(n+p-m)/2+e*(m-p))
                R = objet.d_Aileron/2

                #Calcul des coefficients par surface de calcul

                C_1 = (m_s_a/3)*(((R+e)**2-R**2)*(3*p*(Xa-p)**2/(2*e))+((R+e)**3-R**3)*(Xa-p)**2*p**2/(e**2)+((R+e)**4-R**4)*p**3/(4*e**3))
                A_1 = (m_s_a*p/(4*e))*((R+e)**4-R**4)
                B_1 = A_1 + C_1
                E_1 = (m_s_a/2)*((Xa-p)*p*((R+e)**2-R**2)/e+(p**2/(3*e**2))*((R+e)**3-R**3))

                A_2 = m_s_a*(m-p)*((R+e)**3-R**3)/3
                C_2 = (m_s_a*R/3)*((Xa-p)**3-(Xa-m)**3)
                B_2 = A_2+C_2
                E_2 = (m_s_a/4)*((Xa-p)**2-(Xa-m)**2)*((R+e)**2-R**2)

                A_3 = m_s_a*n*((R+e)**4-R**4)/(4*e)
                C_3 = (m_s_a/3)*(((R+e)**4-R**4)*n**3/(4*e**3) - ((R+e)**3-R**3)*(Xa-m)*n**2/(3*e**2) + ((R+e)**2 - R**2)*(Xa-m)**2*n/(2*e))
                B_3 = A_3 + C_3
                E_3 = (m_s_a/2)*((R**4-(R+e)**4)*n**2/(4*e**2) + ((R+e)**3-R**3)*(Xa-m)*2*n/(3*e))

                #Détermination de la matrice d'inertie d'un ailerons

                Aa = A_1 + A_2 + A_3
                Ba = B_1 + B_2 + B_3
                Ca = C_1 + C_2 + C_3
                Ea = E_1 + E_2 + E_3

                matrice_inertie_un_aileron =  np.array([[Aa ,  0., -Ea],
                                                        [0.  , Ba,   0.],
                                                        [-Ea,  0., Ca ]])
                matrice_inertie_des_ailerons = np.array ([[0., 0., 0.],
                                                          [0., 0., 0.],
                                                          [0., 0., 0.]])
                for i in range(1,objet.nb_ailerons+1):
                    phi = 2*np.pi/i
                    mat_passage = np.array([[1, 0., 0.],
                                            [0., np.cos(phi), np.sin(phi)],
                                            [0., -np.sin(phi), np.cos(phi)]])
                    mat_passage_inv = np.linalg.inv(mat_passage)
                    matrice_inertie_des_ailerons = matrice_inertie_des_ailerons + np.dot(np.dot(mat_passage_inv,matrice_inertie_un_aileron),mat_passage)
                I_CG_ailerons += matrice_inertie_des_ailerons
        return I_CG_ailerons
    
#################################################################################################################################################################################################
#####   Classe des capteurs
#################################################################################################################################################################################################

class Accelerometre :
    """
    Classe des accélérometres 
    
    Prend en argument la position de l'accélérometre

    X : X=[x,y,z] dans le repère de la fusée (l'origine est le nez de la fusée)

    phi, theta, psi : les angles de décalage par rapport au repere de la fusée

    incertitude_phi, incertitude_theta, incertitude_psi : incertitude dans la mesure de la position angulaire de l'accélérometre

    noise_power : amplitude du bruit blanc gaussien

    erreur_de_justesse : le biai de mesure

    """
    def __init__(self ,X ,phi=0.0, theta=0.0, psi=0.0, incertitude_phi=0.0, incertitude_theta=0.0, incertitude_psi=0.0, noise_power=0.0, erreur_de_justesse=0.0):
        #Paramètres de l'accelerometre
        #_________________________
        self.liste_mesures = []
        self.X = X
        self.phi, self.theta, self.psi = phi, theta, psi
        self.incertitude_phi, self.incertitude_theta, self.incertitude_psi = incertitude_phi, incertitude_theta, incertitude_psi 
        self.noise_power, self.erreur_de_justesse = noise_power, erreur_de_justesse
        #_________________________

        #Erreur dans la connaissance de la position de l'accélérometre
        self.phi += np.random.uniform(-self.incertitude_phi,self.incertitude_phi)
        self.theta += np.random.uniform(-self.incertitude_theta,self.incertitude_theta)
        self.psi += np.random.uniform(-self.incertitude_psi,self.incertitude_psi)

        #Calcul du biai de mesure du à l'erreur de justesse
        ax0 = self.erreur_de_justesse*(2*np.random.random()-1)
        ay0 = sqrt(self.erreur_de_justesse**2-ax0**2)*(2*np.random.random()-1)
        az0 = sqrt(self.erreur_de_justesse**2-ax0**2-ay0**2)*(2*np.random.random()-1)
        self.biai = np.array([ax0,ay0,az0])
        np.random.shuffle(self.biai)

    
    def mesure(self, y1, y2, y3, gps, h, X_CG): 
        z, v, q1, p, q, r = y1[2], np.array([y1[3], y1[4], y1[5]]), np.array([y1[6], y1[7], y1[8], y1[9]]), y1[10], y1[11], y1[12]
        if type(y2)!=type(None):
            v2 = np.array([y2[3], y2[4], y2[5]])
            q2 = np.array([y2[6], y2[7], y2[8], y2[9]])
            if type(y3)!=type(None):
                q3 = np.array([y3[6], y3[7], y3[8], y3[9]])
            else:
                q3 = q2
        else:
            v2 = v
            q2 = q1
            q3 = q1
        
         
        ### Calcul de l'accélération dans le repère de la fusée
        a = (v2-v)/h

        ### Ajout de l'accélération de pesanteur dans la mesure
        latitude = gps.position_GPS(y1)[0]
        pesanteur_repere_fusee = np.dot(Mat_fusee_terre(y1), np.array([0, 0, g_acceleration_pesanteur(latitude,z)]))
        a += pesanteur_repere_fusee
        
        ### Calcul des vitesses et accélérations angulaires sur les axes du repère de la fusée
        psi, theta, phi = quat2eul('zyx', q1)
        psi2, theta2, phi2 = quat2eul('zyx', q2)
        psi3, theta3, phi3 = quat2eul('zyx', q3)
        dphi, dtheta, dpsi = derivAngle ([psi, theta, phi], [psi2, theta2, phi2], h)
        dphi2, dtheta2, dpsi2 = derivAngle ([psi2, theta2, phi2], [psi3, theta3, phi3], h)
        
        omegax = dpsi
        omegay = dtheta*np.cos(psi)+dphi*np.cos(theta)*np.sin(psi)
        omegaz = -dtheta*np.sin(psi)-dphi*np.cos(theta)*np.cos(psi)
        omegax2 = dpsi2
        omegay2 = dtheta2*np.cos(psi2)+dphi2*np.cos(theta2)*np.sin(psi2)
        omegaz2 = -dtheta2*np.sin(psi2)-dphi2*np.cos(theta2)*np.cos(psi2)
    
        domegax = (omegax-omegax2)/h
        domegay = (omegay-omegay2)/h
        domegaz = (omegaz-omegaz2)/h

        ### Ajout des accélérations centrifuges mesurées par l'accélérometre dû aux rotations de la fusée
        a[0] += (X_CG-self.X[0])*(omegaz**2 + omegay**2)
        a[1] += self.X[1]*(omegax**2 + omegaz**2)
        a[2] += self.X[2]*(omegay**2 + omegax**2)

        ### Ajout des accélérations linéaires mesurées par l'accélérometre dues à la rotation de la fusée
        a[0] += self.X[2]*domegay - self.X[1]*domegaz
        a[1] += (X_CG-self.X[0])*domegaz - self.X[2]*domegax
        a[2] += self.X[1]*domegax - (X_CG-self.X[0])*domegay
        
        ### Passage dans le repère de l'accélérometre
        mat_change_rep = np.array([[cos(self.psi)*cos(self.theta)                               , sin(self.psi)*cos(self.theta)                               , -sin(self.theta)        ],
                     [-sin(self.psi)*cos(self.phi)+cos(self.psi)*sin(self.theta)*sin(self.phi)   , cos(self.psi) * cos(self.phi)+sin(self.psi)*sin(self.theta)*sin(self.phi)  , cos(self.theta)*sin(self.phi)],
                     [sin(self.psi)*sin(self.phi)+cos(self.psi)*sin(self.theta)*cos(self.phi)    , -cos(self.psi)*sin(self.phi)+sin(self.psi)*sin(self.theta)*cos(self.phi)   , cos(self.theta)*cos(self.phi)]])

        a = np.transpose(mat_change_rep).dot(a)

        ### Ajout du biai et du bruit
        a_mesure = a + self.biai + np.array([self.noise_power*(2*np.random.random()-1) for i in range(3)])
        
        ### Enregistrement de la mesure
        self.liste_mesures.append(a_mesure)

        ### Renvoie du la mesure
        return a_mesure

class Gyroscope :
    """
    Classe des gyroscopes 
    
    Prend en argument la position du gyroscope

    phi, theta, psi : les angles de décalage par rapport au repere de la fusée

    incertitude_phi, incertitude_theta, incertitude_psi : incertitude dans la mesure de la position angulaire du gyroscope

    noise_power : amplitude du bruit blanc gaussien

    erreur_de_justesse : le biai de mesure

    """
    def __init__(self, sequence, phi=0.0, theta=0.0, psi=0.0, incertitude_phi=0.0, incertitude_theta=0.0, incertitude_psi=0.0, noise_power=0.0, erreur_de_justesse=0.0):
        #Paramètres du gyroscope
        #_________________________
        self.liste_mesures = []
        self.sequence = sequence
        self.phi, self.theta, self.psi = phi, theta, psi
        self.incertitude_phi, self.incertitude_theta, self.incertitude_psi = incertitude_phi, incertitude_theta, incertitude_psi 
        self.noise_power, self.erreur_de_justesse = noise_power, erreur_de_justesse
        #_________________________

        #Erreur dans la connaissance de la position du gyroscope
        self.phi += np.random.uniform(-self.incertitude_phi,self.incertitude_phi)
        self.theta += np.random.uniform(-self.incertitude_theta,self.incertitude_theta)
        self.psi += np.random.uniform(-self.incertitude_psi,self.incertitude_psi)

        #Calcul du biai de mesure du à l'erreur de justesse
        phi = self.erreur_de_justesse*(2*np.random.random()-1)
        theta = sqrt(self.erreur_de_justesse**2-phi**2)*(2*np.random.random()-1)
        psi = sqrt(self.erreur_de_justesse**2-phi**2-theta**2)*(2*np.random.random()-1)
        self.biai = np.array([phi,theta,psi])
        np.random.shuffle(self.biai)


    def mesure(self, y): 
        q = np.array([y[6], y[7], y[8], y[9]])
        angles = np.array(quat2eul(self.sequence, q))

        ### Passage dans le repère du gyroscope
        angles += np.array([self.phi, self.theta, self.psi])

        ### Ajout du biai et du bruit
        angles_mesures = angles + self.biai + np.array([self.noise_power*(2*np.random.random()-1) for i in range(3)])
        
        ### Enregistrement de la mesure
        self.liste_mesures.append(angles_mesures)

        ### Renvoie du la mesure
        return angles_mesures

class Gyrometre :
    """
    Classe des gyrometres 
    
    Prend en argument la position du gyrometre

    noise_power : amplitude du bruit blanc gaussien

    erreur_de_justesse : le biai de mesure

    """
    def __init__(self, noise_power=0.0, erreur_de_justesse=0.0):
        #Paramètres du gyroscope
        #_________________________
        self.liste_mesures = []
        self.noise_power, self.erreur_de_justesse = noise_power, erreur_de_justesse
        #_________________________

       
        #Calcul du biai de mesure du à l'erreur de justesse
        


    def mesure(self, y1, y2, h): 
        q1 = np.array([y1[6], y1[7], y1[8], y1[9]])
        angles1 = np.array(quat2eul('zyx', q1))
        phi, theta, psi = angles1[0], angles1[1], angles1[2]
        if (y2 is None):
            q2 = q1
            angles2 = angles1
        else:
            q2 = np.array([y2[6], y2[7], y2[8], y2[9]])
            angles2 = np.array(quat2eul('zyx', q2))
        
        dphi, dtheta, dpsi =derivAngle (angles1, angles2, h)
        
        ### Passage dans le repère du gyrometre
        omegax = dpsi
        omegay = dtheta*np.cos(psi)+dphi*np.cos(theta)*np.sin(psi)
        omegaz = -dtheta*np.sin(psi)-dphi*np.cos(theta)*np.cos(psi)
        
        mesure_parfaite = np.array([omegax, omegay, omegaz])
        ### Ajout du biai et du bruit
        mesure = mesure_parfaite + np.array([self.noise_power*(2*np.random.random()-1) for i in range(3)])
        
        ### Enregistrement de la mesure
        self.liste_mesures.append(mesure)

        ### Renvoie du la mesure
        return mesure


class Barometre :
    """
    Classe des barometres 
    
    Prend en argument les paramètres d'erreur du barometre

    noise_power : amplitude du bruit blanc gaussien

    erreur_de_justesse : le biai de mesure

    """
    def __init__(self, T0 = None, noise_power=0.0, erreur_de_justesse=0.0):
        #Paramètres du barometre
        #_________________________
        self.liste_mesures = []
        self.noise_power, self.erreur_de_justesse = noise_power, erreur_de_justesse
        self.T0 = T0
        #_________________________


    def mesure(self, y): 
        z = y[2]
        P = Pression(z,Temperature(z, self.T0))

        ### Ajout du biai et du bruit
        P_mesures = P + self.erreur_de_justesse + self.noise_power*(2*np.random.random()-1) 
        
        ### Enregistrement de la mesure
        self.liste_mesures.append(P_mesures)

        ### Renvoie du la mesure
        return P_mesures

class Thermometre :
    """
    Classe des thermometres 
    
    Prend en argument les paramètres d'erreur du thermometre

    noise_power : amplitude du bruit blanc gaussien

    erreur_de_justesse : le biai de mesure

    """
    def __init__(self, T0=None, noise_power=0.0, erreur_de_justesse=0.0):
        #Paramètres du thermometre
        #_________________________
        self.liste_mesures = []
        self.noise_power, self.erreur_de_justesse = noise_power, erreur_de_justesse
        self.T0=T0
        #_________________________


    def mesure(self, y): 
        z = y[2]
        T = Temperature(z, self.T0)

        ### Ajout du biai et du bruit
        T_mesures = T + self.erreur_de_justesse + self.noise_power*(2*np.random.random()-1) 
        
        ### Enregistrement de la mesure
        self.liste_mesures.append(T_mesures)

        ### Renvoie du la mesure
        return T_mesures

class Magnetometre :
    """
    Classe des magnétometre 
    
    Prend en argument les paramètres d'erreur du magnétometre

    noise_power : amplitude du bruit blanc gaussien

    erreur_de_justesse : le biai de mesure

    """
    def __init__(self, noise_power=0.0, erreur_de_justesse=0.0, old=False):
        #Paramètres du magnétometre
        #_________________________
        self.liste_mesures = []
        self.noise_power, self.erreur_de_justesse = noise_power, erreur_de_justesse
        self.old = old
        #_________________________

    
    def mesure(self, y, gps, fusee=True): 
        ermessage = False
        if self.old : 
            Bh, Bv = 23,39.84
            B_mesures = np.dot(Mat_fusee_terre(y), np.array([Bh,0,Bv]))
        else:
            X = [y[0],y[1],y[2]]
            lat, lon, alt = gps.position_GPS(X)
            if sys.platform.startswith('win32'):
                if not(ermessage):
                    #print('pas tester avec Windows\n')
                    ermessage = True
                    #print("/!\ veuillez tester avec l'argument old=True /!\ \n")
                addrcodec = '.\WMM2020\src\MagnetoCompiledWindows.exe'
            elif sys.platform.startswith('linux'):
                addrcodec = './WMM2020/src/MagnetoCompiledLinux'
            elif sys.platform.startswith('darwin'):
                if not(ermessage):
                    print('Le programme n a pas été testé sous Mac Os\n Il peut y avoir des problèmes avec le magnétomètre\n')
                    ermessage = True
                    print("/!\ veuillez tester avec l'argument old=True /!\ \n")
            else :
                if not(ermessage):
                    print('Os non reconnu\n Passer le ')
                    ermessage = True
                    print("/!\ veuillez tester avec l'argument old=True /!\ \n")
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

        ### Ajout du biai et du bruit
        ##B_mesures = B + self.erreur_de_justesse + self.noise_power*(2*np.random.random()-1) 
        if fusee:
            B = np.array([B_mesures[2],B_mesures[3],B_mesures[4]])
            B_xyz = np.dot(Mat_fusee_terre(y), B)
            B_mesures[2] = B_xyz[0]
            B_mesures[3] = B_xyz[1]
            B_mesures[4] = B_xyz[2]

        ### Enregistrement de la mesure
        self.liste_mesures.append(B_mesures)

        ### Renvoie du la mesure
        return B_mesures

class GPS :
    """
    Classe des GPS 
    
    Prend en argument les paramètres d'erreur du GPS

    noise_power : amplitude du bruit blanc gaussien

    erreur_de_justesse : le biai de mesure

    """
    def __init__(self, noise_power=0.0, erreur_de_justesse=0.0):
        #Paramètres du GPS
        #_________________________
        self.liste_mesures = []
        self.noise_power, self.erreur_de_justesse = noise_power, erreur_de_justesse
        #_________________________


    def mesure(self, y, gps): 
        """
        Paramètres : 
        - y : le vecteur état de la fusée
        - gps : l'object permettant de calculer la position gps de la fusée
        Retourne la position gps et enregistre la mesure
        """
        X = [y[0],y[1],y[2]]
        lat, lon, alt = gps.position_GPS(X)
        GPS_mesures = [lat, lon, alt]
        ### Enregistrement de la mesure
        self.liste_mesures.append(GPS_mesures)

        ### Renvoie du la mesure
        return GPS_mesures

def derivAngle (angles1, angles2, h):
    if abs((angles1[0]-angles2[0])/h)<3:
        dphi = (angles1[0]-angles2[0])/h
    elif angles1[0]>angles2[0]:
        dphi = (angles1[0] - 2*np.pi -angles2[0])/h
    else :
        dphi = (angles1[0] + 2*np.pi -angles2[0])/h

    if abs((angles1[1]-angles2[1])/h)<3:
        dtheta = (angles1[1]-angles2[1])/h
    elif angles1[1]>angles2[1]:
        dtheta = (angles1[1] - 2*np.pi -angles2[1])/h
    else :
        dtheta = (angles1[1] + 2*np.pi -angles2[1])/h
    
    if abs((angles1[2]-angles2[2])/h)<3:
        dpsi = (angles1[2]-angles2[2])/h
    elif angles1[2]>angles2[2]:
        dpsi = (angles1[2] - 2*np.pi -angles2[2])/h
    else :
        dpsi = (angles1[2] + 2*np.pi -angles2[2])/h
    
    return dphi, dtheta, dpsi
