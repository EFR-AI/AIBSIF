from numpy import cos, sin, tan, 
import numpy as np


def Mat_fusee_terre_cardan(X):
    """
    paramètre : vecteur X = (x,y,z,vx,vy,vz,phi,theta,psi,p,q,r)
    Matrice de changement de base entre le repère terrestre et le repere de la fusee
    """
    phi, theta, psi = X[6], X[7], X[8]
    return np.array([[cos(psi)*cos(theta)                               , sin(psi)*cos(theta)                               , -sin(theta)        ],
                     [-sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi)   , cos(psi) * cos(phi)+sin(psi)*sin(theta)*sin(phi)  , cos(theta)*sin(phi)],
                     [sin(psi)*sin(phi)+cos(psi)*sin(theta)*cos(phi)    , -cos(psi)*sin(phi)+sin(psi)*sin(theta)*cos(phi)   , cos(theta)*cos(phi)]],)


def Mat_deriv_cardan(X):
    [phi,theta] = X[6:8]
    return np.array([[1, sin(phi)*tan(theta), cos(phi)*tan(theta)],
                          [0, cos(phi), -sin(phi)],
                          [0, sin(phi)/cos(theta), cos(phi)/cos(theta)]])



"""
def F_air_cardan(X, t,Parameters):
    
    #mouvement de la fusée dans les airs
    #F de l'équation differentielle :
    #y' = F(t,y)
    #avec Vecteur X = (x,y,z,vx,vy,vz,phi,theta,psi,p,q,r)
    #t temps
    #Parameters = parametres physiques =  [masse, dm, Poussee, Cx0, Cn_alpha,  Marge_static, Surface, rho, Inertie, Inv_inertie] 
    
    
    [ub, vb, wb, phi, theta, psi, p, q, r] = X[3:]
    Vfusee = np.array([ub, vb, wb])
    V = Vitesse_relative(X, Vfusee)

    [masse, dm, Poussee, Cx0, Cn_alpha, Marge_static, Surface, rho, Inertie, Inv_inertie] = Parameters

    Cx = Cx_v(Cx0,linalg.norm(V),X)

    Mat_deriv = np.array([[1, sin(phi)*tan(theta), cos(phi)*tan(theta)],
                          [0, cos(phi), -sin(phi)],
                          [0, sin(phi)/cos(theta), cos(phi)/cos(theta)]],)

    Mat_rot = np.array([[0, -r, q],
                        [r, 0, -p],
                        [-q, p, 0]],)

    [dphi, dtheta, dpsi] = Mat_deriv.dot(np.array([p, q, r]))

    # PFD dans un référentiel non galiléen
    Acceleration = 1/masse * (Poid(masse, X)+Force_Poussee(Poussee) + Force_aero(V, rho, Cx, Cn_alpha,Surface) - dm*Vfusee)-Mat_rot.dot(Vfusee) 
    [dub, dvb, dwb] = Acceleration

    # théorème du moment cinétique dans un référentiel non galiléen
    Moments = Moment_aero(V, rho, Cx, Cn_alpha, Surface, Marge_static) - Mat_rot.dot(Inertie).dot(np.array([p, q, r])) - dm*Inertie.dot(np.array([p, q, r]))
    [dp, dq, dr] = Inv_inertie.dot(Moments)

    [dx, dy, dz] = np.transpose(Mat_fusee_terre(X)).dot(Vfusee)
    res = np.array([dx, dy, dz, dub, dvb, dwb, dphi, dtheta, dpsi, dp, dq, dr])
    res[abs(res)<pow(10,-10)] = 0 # cos(pi/2) != 0, pour avoir une fusée parfaitement verticale sinon l'erreur se propage 

    return res
    """