from numpy import cos, sin ,tan, arccos, arctan,arcsin 
import numpy.linalg as linalg
import numpy as np 
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from rockets.astreos import MyRocket


def Pression_amb_isotherme(X):
   """
   modèle d'une athmosphere isotherme 
   """
   h = -X[1]
   H0 = 8000 #m = R*T0/(Mair*g)
   P0= 1013.25 *pow(10,2)# Pa
   return np.round(P0*np.exp(-h/H0),5)

def rho_air(X,T=298.15):  
   """
   loi des gaz parfait 
   si pas de Temperature : T0 = 298.1 K = 25°C 
   """
   M = 28.965 *pow(10,-3)# kg.mol-1 à 1 bar 
   R = 8.314 #constante des gaz parfait
   return Pression_amb_isotherme(X)*M/(R*T)


def correct_erreur(X):
    L= []
    for a in X:
        if abs(a)<pow(10,-10):
            L.append(0)
        else:
            L.append(a)
    return np.array(L)



def Cx_v(NormeV):
    V = NormeV
    if V<= 200:
        return Cx0
    if V<= 225:
        return Cx0*1.14
    if V<= 250:
        return Cx0*1.36
    if V<= 275:
        return Cx0*1.91
    if V<= 300:
        return Cx0*2.77
    if V<= 325:
        return Cx0*2.59
    if V<= 350:
        return Cx0*2.27
    if V<= 375:
        return Cx0*1.91
    if V<= 400:
        return Cx0*1.77
    if V<= 425:
        return Cx0*1.68
    if V<= 450:
        return Cx0*1.59
    else :
        return Cx0*1.55


def F_air_3(X,t):
    """
    F de l'équation differentielle :
    y' = F(t,y)
    avec y = (x,z,vx,vz,theta,q)
    """
    [ub,wb,theta,q]= X[2:]
    #print([ub,wb,theta,q])
    Norme =np.sqrt(ub**2+wb**2)
    Cx= Cx_v(Norme)
    if Norme <= pow(10,-10):
        Cz = 0
    else :
        Cz = Cn_alpha * arcsin(wb/Norme)
    Fx = -1/2*rho*Surface*Cx*np.sign(ub)*Norme**2
    Fz = -1/2*rho*Surface*Cz*Norme**2
    M =  -1/2*rho*Surface*Cz*Norme**2* abs(X_CPA-X_CG)/100
    dub = 1/masse*(Fx+Poussee)-g*sin(theta)-q*wb
    dwb = 1/masse*Fz+ q*ub+g*cos(theta) 
    print(Fz,Cz,Norme)
    dq = M/Inertie
    
    dtheta = q
    """
    if dub <= 0:
        dub = 0"""

    res = np.array([ub*cos(theta)+wb*sin(theta),-ub*sin(theta)+wb*cos(theta),dub,dwb,dtheta,dq])
  
    #print([ub*cos(theta)+wb*sin(theta),-ub*sin(theta)+wb*cos(theta),dub,dwb,dtheta,dq])
    return res


def Integration_Runge_Kutta(f,y,t,h):
    k1 = f(y, t)
    k2 = f(y+h/2*k1, t+h/2)
    k3 = f(y+h/2*k2, t+h/2)
    k4 = f(y+h*k3, t)

    return  y + h/6*(k1 + 2*k2 + 2*k3 + k4)

def Integration_Euler(f,y,t,h):
    return  y + h*f(y, t)

def trajectoire_3ddl(h,y0):
    """
    Attention athmosphère
    """
    global masse,g,V_vent,Cx0,Cn_alpha,Surface,rho,X_CPA,X_CG,Poussee,Hauteur,Inertie,eps,alpha,beta

    
    my_rocket = MyRocket()
    # parametres
    Hauteur= my_rocket.fusee.H/100 # conversion en m 
    # vent horizontal à la surface de la terre 
   
    masse =my_rocket.fusee.masse

    g = 9.81
    
    Cx0 = 0.6
    Cn_alpha = my_rocket.fusee.Cn_alpha
    
    Diametre = my_rocket.fusee.diametre/100 # conversion en m 
    Surface = np.pi * Diametre**2 /4 

    X_CPA = my_rocket.fusee.X_CPA/100  # conversion en m 
    X_CG = my_rocket.fusee.X_CG/100 # conversion en m 
    print(X_CPA-X_CG, X_CPA,X_CG,Cn_alpha)
   
    Iy = (Diametre/2)**2/4+1/12*Hauteur**2
    Inertie =  masse*Iy
    

    t = [0]
    y = [y0]

    print(my_rocket.fusee.temps_pousse)
    i=0
    f = F_air_3
    while i <= nb_iter_max:
        #print(round(i*h,2), y[-1][7])
        i+=1 
        if i*h < 0.1:
            Poussee= 760.3*(i*h/0.1)
        elif i*h < 0.2:
            Poussee= 760.3 + 1200*(i*h-0.1)/0.1
        elif i*h < 0.3:
            Poussee = 1200 + 2500*(i*h-0.2)/0.1
        elif i*h <0.4 :
            Poussee = 2500 + 5000*(i*h-0.3)/0.1
        elif i*h < my_rocket.fusee.temps_pousse:
            Poussee= 5000
        elif i*h < (my_rocket.fusee.temps_pousse + 0.1) :
            Poussee= 5000 - 5000*(i*h-(my_rocket.fusee.temps_pousse + 0.1))/0.1
            masse = masse - my_rocket.fusee.masse_ergol*h/my_rocket.fusee.temps_pousse
        else :
            Poussee= 0
            masse = my_rocket.fusee.masse_vide
        rho = rho_air(y[-1])
        #print(rho)
        """
        if f == F_rampe and -y[-1][2] > Longeur_rampe*sin(alpha):
            print(i)
            f = F_air_3
        """
        """
        if np.transpose(Mat_fusee_terre(y[-1])).dot(np.array([y[-1][3],y[-1][4],y[-1][5]])).dot(np.array([0,0,1]))>= 0  and y[-1][2]<= 50 :
            #print(i*h)
            #print(Surface)
            Surface = np.pi * 2**2 /4 
            Cx0 = 1.5"""
            

        
        
        
        y.append(Integration_Euler(f,y[-1],t[-1],h))
        t.append(i*h)
  
        
    return t,y


V_vent = 0
eps = 0
Longeur_rampe = 6
alpha = np.pi/3
h = 0.1

X0 = np.array([1,2.5,1,0.333,alpha,1])
y0 = np.array([1,0,2.5,1,0,0.333,0,alpha,0,0,1,0])

my_rocket = MyRocket()# parametres
Hauteur= my_rocket.fusee.H/100 # conversion en m 
masse =my_rocket.fusee.masse
g = 9.81
Cx0 = 0.6
Cn_alpha = my_rocket.fusee.Cn_alpha
Diametre = my_rocket.fusee.diametre/100  # conversion en m
Surface = np.pi * Diametre**2 / 4
X_CPA = my_rocket.fusee.X_CPA/100
X_CG = my_rocket.fusee.X_CG/100  # conversion en m
Iy = (Diametre/2)**2/4+1/12*Hauteur**2
Inertie = masse*Iy
rho = rho_air(X0)
Poussee= 0

print("Finesse",Hauteur/Diametre)
print("Marge Statique",(X_CPA-X_CG)/Diametre )
print("Cnalpha",Cn_alpha)
print("produit Cn_alpha*MS",Cn_alpha*(X_CPA-X_CG))


print([i for i in F_air_3(X0,0)])


"""
nb_iter_max= 3
# attention cos(pi/2) = 6 *10^(-17)-> source d'erreur 
V_vent = 0
eps = 0
Longeur_rampe = 6
alpha = np.pi/3
beta = 0
y0 = np.array([0,0,1,0,alpha,0])
h = 0.05
t,Y =trajectoire_3ddl(h,y0)

n = 1

list_z = np.array([Y[i][1] for i in range(0,len(Y),n)])
list_x = np.array([Y[i][0] for i in range(0,len(Y),n)])


plt.plot(list_x,-list_z)
#plt.plot(t,list_x)
plt.xlabel("x")
plt.ylabel("alitutude (z)")
plt.title('trajectoire')
#plt.plot(t,list_vz)
plt.show()

"""
