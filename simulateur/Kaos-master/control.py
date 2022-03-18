from numpy import cos, sin, arcsin, arctan
from numpy import linalg
import numpy as np

from quaternion import (
    Mat_deriv_quaternion,
    Mat_fusee_terre,
    cardan_to_quaternion,
    quaternion_to_cardan,
)


def sgn(x):
    if x >= 0:
        return 1
    return -1

def Force_Poussee(P,deltr,deltq):
    """
    paramètre : Norme de la force de Poussee
    force : Vecteur Poussee du moteur dans le repère de la fusée
    """
    M = np.array(
        [
            [cos(deltr) * cos(deltq)    , -sin(deltr)   , cos(deltr) * sin(deltq)],
            [sin(deltr) * cos(deltq)    , cos(deltr)    , sin(deltr) * sin(deltq)],
            [-sin(deltq)                , 0             , cos(deltq)             ],
        ]
    )
    return np.dot( M, np.array([P, 0, 0]))

def Vitesse_relative(X, V_abs, puissance_vent, Vect_vent):
    """
    paramètre : vecteur X = (x,y,z,vx,vy,vz,q0,q1,q2,q3,p,q,r)
    Vitesse relative de la fusee : vitesse de la fusee -vitesse  vent
    """
    Vent = Mat_fusee_terre(X).dot(Vect_vent) * puissance_vent

    return -np.array([-V_abs[0] + Vent[0], -V_abs[1] + Vent[1], -V_abs[2] + Vent[2]])


def Force_aero(V, rho, C_A, Cn_alpha, Surface):
    """
    #paramètre : V Vecteur vitesse, rho densite de l'air, CX0 coefficient de trainée, Cn_alpha gradient de portance
    #Force aerodynamique : Vecteur Portance et Traine
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
            -1 / 2 * rho * Surface *  Norme ** 2 * np.array([C_A, C_Y, C_N]) 
        )


def Moment_aero(X,V, rho,Cn_alpha, coeff, Surface, l_ref,l_d):
    """
    #paramètre : V Vecteur vitesse, rho densite de l'air
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

#print(Force_aero([1,1,-2],1.2, 0.5,1,1))
#print(Moment_aero([1,-1,2],1.2,0.5,1,1,10))

class Controler:
    def __init__(self, nb_i_delais, quaternion_desire, debattement):
        self.nb_i_delais = nb_i_delais
        self.next_answer = 0, 0
        self.i0 = 0
        self.quaternion_desire = quaternion_desire
        self.debattement = debattement

    def theta_constant(self, i, X, Parameters):

        if i - self.i0 >= self.nb_i_delais:
            self.i0 = i
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
                deltq,
            ] = Parameters

            [l_d,l_ref,l_p,l_o_xcg] = Distance

            [ub, vb, wb, q0, q1, q2, q3, p, q, r] = X[3:]
            Vfusee = np.array([ub, vb, wb])
            V = Vitesse_relative(X, Vfusee, puissance_vent, Vect_vent)

            # Normalisation des quaternions pour la stabilité
            N = q0 ** 2 + q1 ** 2 + q2 ** 2 + q3 ** 2
            q0, q1, q2, q3 = q0 / N, q1 / N, q2 / N, q3 / N

            Mat_rot = np.array(
                [[0, -r, q], [r, 0, -p], [-q, p, 0]],
            )

            phid, psid = (
                quaternion_to_cardan([q0, q1, q2, q3])[0],
                quaternion_to_cardan([q0, q1, q2, q3])[2],
            )
            thetad = quaternion_to_cardan(self.quaternion_desire)[1]
            [q0d, q1d, q2d, q3d] = cardan_to_quaternion(phid, thetad, psid)

            # Gains : 38/10000
            K1 = 38 * np.eye(3) #38
            K2 = 500 * np.eye(3) #10000

            omega_tilde = np.array([p, q, r])

            # le quaternion d'erreur: q_desirée x q*
            q_tilde = np.array(
                [
                    q0 * q0d + q1 * q1d + q2 * q2d + q3 * q3d,
                    -q1 * q0d + q0 * q1d + q2 * q3d - q3 * q2d,
                    -q2 * q0d + q0 * q2d - q1 * q3d + q3 * q1d,
                    -q3 * q0d + q0 * q3d + q1 * q2d - q2 * q1d,
                ]
            )

            F_aero = Force_aero(V, rho, Cx, Cn_alpha, Surface)
            F_Poussee = Force_Poussee(Poussee,deltr,deltq)
            M_aero = Moment_aero(X,V, rho, Cn_alpha,Coeff, Surface, l_ref,l_d) + np.cross(l_o_xcg*np.array([-1,0,0]),F_aero)

            #print(Force_aero(V, rho, Cx, Cn_alpha, Surface),Moment_aero(V, rho, Cx, Cn_alpha, Surface, Marge_static))
            # Somme des moments exceptés ceux des actionneurs
            tau_d = (
                M_aero
                - dm * Inertie.dot(np.array([p, q, r]))
                - Mat_rot.dot(Inertie).dot(np.array([p, q, r]))
            )

            z1 = np.array([1 - abs(q_tilde[0]), q_tilde[1], q_tilde[2], q_tilde[3]])

            G_z1 = sgn(q_tilde[0]) * np.array([q_tilde[1], q_tilde[2], q_tilde[3]])

            alpha1 = -K1.dot(G_z1)

            dq_tilde = Mat_deriv_quaternion(omega_tilde).dot(q_tilde)

            G_t = np.array(
                [
                    [
                        q_tilde[1] * sgn(q_tilde[0]),
                        q_tilde[2] * sgn(q_tilde[0]),
                        q_tilde[3] * sgn(q_tilde[0]),
                    ],
                    [q_tilde[0], -q_tilde[3], q_tilde[2]],
                    [q_tilde[3], q_tilde[0], -q_tilde[1]],
                    [-q_tilde[2], -q_tilde[1], q_tilde[0]],
                ]
            )

            dG = np.array(
                [
                    [
                        dq_tilde[1] * sgn(q_tilde[0]),
                        dq_tilde[2] * sgn(q_tilde[0]),
                        dq_tilde[3] * sgn(q_tilde[0]),
                    ],
                    [dq_tilde[0], -dq_tilde[3], dq_tilde[2]],
                    [dq_tilde[3], dq_tilde[0], -dq_tilde[1]],
                    [-dq_tilde[2], -dq_tilde[1], dq_tilde[0]],
                ]
            )

            dalpha1 = -K1.dot(
                np.transpose(dG).dot(z1)
                + 1 / 2 * np.transpose(G_t).dot(G_t).dot(omega_tilde)
            )

            z2 = omega_tilde - alpha1

            T = G_z1 + K2.dot(z2) + tau_d - Inertie.dot(dalpha1)

            T = T -  np.cross(l_o_xcg*np.array([-1,0,0]),F_Poussee)
            
            # On ne renvoie pas directement T mais deltq et deltr
            # pour pouvoir controler le debattement

            if T[1] / Poussee / l_p < -1:
                deltq = -np.pi / 2
            elif T[1] / Poussee / l_p > 1:
                deltq = np.pi / 2
            else:
                deltq = arcsin(T[1] / Poussee / l_p)

            if T[2] / Poussee / l_p / cos(deltq) < -1:
                deltr = -np.pi / 2
            elif T[2] / Poussee / l_p / cos(deltq) > 1:
                deltr = np.pi / 2
            else:
                deltr = arcsin(T[2] / Poussee / l_p / cos(deltq))

            if deltq > self.debattement / 180 * np.pi:
                deltq = self.debattement / 180 * np.pi
                # print("Debattement de la tuyere atteint")
            elif deltq < -self.debattement / 180 * np.pi:
                deltq = -self.debattement / 180 * np.pi
                # print("Debattement de la tuyere atteint")

            if deltr > self.debattement / 180 * np.pi:
                deltr = self.debattement / 180 * np.pi
                # print("Debattement de la tuyere atteint")
            elif deltr < -self.debattement / 180 * np.pi:
                deltr = -self.debattement / 180 * np.pi
                # print("Debattement de la tuyere atteint")

            if self.nb_i_delais > 0:
                answerq, answerr = self.next_answer
                self.next_answer = deltq, deltr
            else:
                answerq, answerr = deltq, deltr

            return True, (answerq, answerr)
        return False, []

    def quaternion_constant(self, i, X, Parameters):

        if i - self.i0 >= self.nb_i_delais:
            self.i0 = i
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
                deltq,
            ] = Parameters

            [l_d,l_ref,l_p,l_o_xcg] = Distance

            [ub, vb, wb, q0, q1, q2, q3, p, q, r] = X[3:]
            Vfusee = np.array([ub, vb, wb])
            V = Vitesse_relative(X, Vfusee, puissance_vent, Vect_vent)

            # Normalisation des quaternions pour la stabilité
            N = q0 ** 2 + q1 ** 2 + q2 ** 2 + q3 ** 2
            q0, q1, q2, q3 = q0 / N, q1 / N, q2 / N, q3 / N

            Mat_rot = np.array(
                [[0, -r, q], 
                [r, 0, -p], 
                [-q, p, 0]],
            )

            [q0d, q1d, q2d, q3d] = self.quaternion_desire

            # Gains : pas de delais  90/800
            # délais de 2 15/3000
            K1 = 15 * np.eye(3)
            K2 = 3000 * np.eye(3)

            omega_tilde = np.array([p, q, r])

            # le quaternion d'erreur: q_desirée x q*
            q_tilde = np.array(
                [
                    q0 * q0d + q1 * q1d + q2 * q2d + q3 * q3d,
                    -q1 * q0d + q0 * q1d + q2 * q3d - q3 * q2d,
                    -q2 * q0d + q0 * q2d - q1 * q3d + q3 * q1d,
                    -q3 * q0d + q0 * q3d + q1 * q2d - q2 * q1d,
                ]
            )

            F_aero = Force_aero(V, rho, Cx, Cn_alpha, Surface)
            F_Poussee = Force_Poussee(Poussee,deltr,deltq)
            M_aero = Moment_aero(X,V, rho, Cn_alpha,Coeff, Surface, l_ref,l_d) + np.cross(l_o_xcg*np.array([-1,0,0]),F_aero)


            # Somme des moments exceptés ceux des actionneurs
            tau_d = (
                M_aero
                - dm * Inertie.dot(np.array([p, q, r]))
                - Mat_rot.dot(Inertie).dot(np.array([p, q, r]))
            )

            z1 = np.array([1 - abs(q_tilde[0]), q_tilde[1], q_tilde[2], q_tilde[3]])

            G_z1 = sgn(q_tilde[0]) * np.array([q_tilde[1], q_tilde[2], q_tilde[3]])

            alpha1 = -K1.dot(G_z1)

            dq_tilde = Mat_deriv_quaternion(omega_tilde).dot(q_tilde)

            G_t = np.array(
                [
                    [
                        q_tilde[1] * sgn(q_tilde[0]),
                        q_tilde[2] * sgn(q_tilde[0]),
                        q_tilde[3] * sgn(q_tilde[0]),
                    ],
                    [q_tilde[0], -q_tilde[3], q_tilde[2]],
                    [q_tilde[3], q_tilde[0], -q_tilde[1]],
                    [-q_tilde[2], -q_tilde[1], q_tilde[0]],
                ]
            )

            dG = np.array(
                [
                    [
                        dq_tilde[1] * sgn(q_tilde[0]),
                        dq_tilde[2] * sgn(q_tilde[0]),
                        dq_tilde[3] * sgn(q_tilde[0]),
                    ],
                    [dq_tilde[0], -dq_tilde[3], dq_tilde[2]],
                    [dq_tilde[3], dq_tilde[0], -dq_tilde[1]],
                    [-dq_tilde[2], -dq_tilde[1], dq_tilde[0]],
                ]
            )

            dalpha1 = -K1.dot(
                np.transpose(dG).dot(z1)
                + 1 / 2 * np.transpose(G_t).dot(G_t).dot(omega_tilde)
            )

            z2 = omega_tilde - alpha1

            T = G_z1 + K2.dot(z2) + tau_d - Inertie.dot(dalpha1)

            T = T -  np.cross(l_o_xcg*np.array([-1,0,0]),F_Poussee)
            # On ne renvoie pas directement T mais deltq et deltr
            # pour pouvoir controler le debattement

            if T[1] / Poussee / l_p < -1:
                deltq = -np.pi / 2
            elif T[1] / Poussee / l_p > 1:
                deltq = np.pi / 2
            else:
                deltq = arcsin(T[1] / Poussee / l_p)

            if T[2] / Poussee / l_p / cos(deltq) < -1:
                deltr = -np.pi / 2
            elif T[2] / Poussee / l_p / cos(deltq) > 1:
                deltr = np.pi / 2
            else:
                deltr = arcsin(T[2] / Poussee / l_p / cos(deltq))

            if deltq > self.debattement / 180 * np.pi:
                deltq = self.debattement / 180 * np.pi
                # print("Debattement de la tuyere atteint")
            elif deltq < -self.debattement / 180 * np.pi:
                deltq = -self.debattement / 180 * np.pi
                # print("Debattement de la tuyere atteint")

            if deltr > self.debattement / 180 * np.pi:
                deltr = self.debattement / 180 * np.pi
                # print("Debattement de la tuyere atteint")
            elif deltr < -self.debattement / 180 * np.pi:
                deltr = -self.debattement / 180 * np.pi
                # print("Debattement de la tuyere atteint")

            if self.nb_i_delais > 0:
                answerq, answerr = self.next_answer
                self.next_answer = deltq, deltr
            else:
                answerq, answerr = deltq, deltr

            return True, (answerq, answerr)
        return False, []

    def reception(self, i, X, Parameters):
        return self.theta_constant(i, X, Parameters)