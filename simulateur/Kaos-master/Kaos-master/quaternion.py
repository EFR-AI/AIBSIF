from numpy import cos, sin, arccos, arcsin, arctan2
import numpy as np
from math import pi

def cardan_to_quaternion(phi, theta, psi):
    """
    conversion des angles de cardan vers le quaternion equivalent
    """
    q0 = cos(phi / 2) * cos(theta / 2) * cos(psi / 2) + sin(phi / 2) * sin(
        theta / 2) * sin(psi / 2)
    q1 = sin(phi / 2) * cos(theta / 2) * cos(psi / 2) - cos(phi / 2) * sin(
        theta / 2) * sin(psi / 2)
    q2 = cos(phi / 2) * sin(theta / 2) * cos(psi / 2) + sin(phi / 2) * cos(
        theta / 2) * sin(psi / 2)
    q3 = cos(phi / 2) * cos(theta / 2) * sin(psi / 2) - sin(phi / 2) * sin(
        theta / 2) * cos(psi / 2)
    return q0, q1, q2, q3


def quaternion_to_cardan(X):
    """
    conversion du quaternion vers les angles de cardan équivalent
    X = (q0,q1,q2,q3)
    tentative de gestion des cas extremal (pas optimal)
    """

    [q0, q1, q2, q3] = X

    # Normalisation des quaternions
    N = q0 ** 2 + q1 ** 2 + q2 ** 2 + q3 ** 2
    q0, q1, q2, q3 = q0 / N, q1 / N, q2 / N, q3 / N

    c = q0 * q2 - q1 * q3
    c = np.round(c, 8)
    if c <= -0.5:
        theta = -np.pi / 2
        phi = 0
        psi = -2 * arccos(2 * q0 / np.sqrt(2))
    elif c >= 0.5:
        theta = np.pi / 2
        phi = 0
        psi = -2 * arccos(2 * q0 / np.sqrt(2))
    else:
        theta = arcsin(2 * (c))
        a = q0 * q1 + q2 * q3
        b = q2 * q1 + q0 * q3
        phi = arctan2(2 * (a), (q0 ** 2 - q1 ** 2 - q2 ** 2 + q3 ** 2))
        psi = arctan2(2 * (b), (q0 ** 2 + q1 ** 2 - q2 ** 2 - q3 ** 2))
    angle = np.array([phi, theta, psi])
    angle[abs(angle) < pow(10, -10)] = 0
    return angle

def singularitycheck(group, theta):
    if (group==1 and pi-theta<pi/180):
        print(f"singularity check failed: {pi-theta} || {theta} < rad\n")
    elif (group==2 and abs(theta-pi/2)<pi/180):
        print(f"singularity check failed: {abs(theta-pi/2)} < {pi/180} rad\n")
    elif (group != 1 and group != 2):
        print("the group must be 1 or 2")

def quat2eul(sequence, q):
    """
    Renvoie les angles d'euler pour les quaternions en entrée (liste q des quaternions) suivant la sequence de rotation (string sequence des trois axes de rotation)
    exemple : quat2eul("xyx", [0.0, 0.1, 0.0, 0.0])
    """
    psi = 0
    theta = 0
    phi = 0

    if ("xyx" == sequence):
        psi = np.arctan2((q[1] * q[2] + q[3] * q[0]), (q[2] * q[0] - q[1] * q[3]))
        theta = np.arccos(q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3])
        phi = np.arctan2((q[1] * q[2] - q[3] * q[0]), (q[1] * q[3] + q[2] * q[0]))
        singularitycheck(1, theta)
    elif ("yzy" == sequence) :
        psi = np.arctan2((q[1] * q[0] + q[2] * q[3]), (q[3] * q[0] - q[1] * q[2]))
        theta = np.arccos(q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3])
        phi = np.arctan2((q[2] * q[3] - q[1] * q[0]), (q[1] * q[2] + q[3] * q[0]))
        singularitycheck(1, theta)
    elif ("zxz" == sequence) :
        psi = np.arctan2((q[1] * q[3] + q[2] * q[0]), (q[1] * q[0] - q[2] * q[3]))
        theta = np.arccos(q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3])
        phi = np.arctan2((q[1] * q[3] - q[2] * q[0]), (q[1] * q[0] + q[2] * q[3]))
        singularitycheck(1, theta)
    elif ("xzx" == sequence) :
        psi = np.arctan2((q[1] * q[3] - q[2] * q[0]), (q[1] * q[2] + q[3] * q[0]))
        theta = np.arccos(q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3])
        phi = np.arctan2((q[1] * q[3] + q[2] * q[0]), (q[3] * q[0] - q[1] * q[2]))
        singularitycheck(1, theta)
    elif ("yxy" == sequence) :
        psi = np.arctan2((q[1] * q[2] - q[3] * q[0]), (q[1] * q[0] + q[2] * q[3]))
        theta = np.arccos(q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3])
        phi = np.arctan2((q[1] * q[2] + q[3] * q[0]), (q[1] * q[0] - q[2] * q[3]))
        singularitycheck(1, theta)
    elif ("zyz" == sequence) :
        psi = np.arctan2((q[2] * q[3] - q[1] * q[0]), (q[1] * q[3] + q[2] * q[0]))
        theta = np.arccos(q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3])
        phi = np.arctan2((q[1] * q[0] + q[2] * q[3]), (q[2] * q[0] - q[1] * q[3]))
        singularitycheck(1, theta)
    elif ("xyz" == sequence) :
        psi = np.arctan2(2 * (q[1] * q[0] - q[2] * q[3]), (q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]))
        theta = np.arcsin(2 * (q[1] * q[3] + q[2] * q[0]))
        phi = np.arctan2(2 * (q[3] * q[0] - q[1] * q[2]), (q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]))
        singularitycheck(2, theta)
    elif ("yzx" == sequence) :
        psi = np.arctan2(2 * (q[2] * q[0] - q[1] * q[3]), (q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]))
        theta = np.arcsin(2 * (q[1] * q[2] + q[3] * q[0]))
        phi = np.arctan2(2 * (q[1] * q[0] - q[3] * q[2]), (q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3]))
        singularitycheck(2, theta)
    elif ("zxy" == sequence) :
        psi = np.arctan2(2 * (q[3] * q[0] - q[1] * q[2]), (q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3]))
        theta = np.arcsin(2 * (q[1] * q[0] + q[2] * q[3]))
        phi = np.arctan2(2 * (q[2] * q[0] - q[3] * q[1]), (q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]))
        singularitycheck(2, theta)
    elif ("xzy" == sequence) :
        psi = np.arctan2(2 * (q[1] * q[0] + q[2] * q[3]), (q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3]))
        theta = np.arcsin(2 * (q[3] * q[0] - q[1] * q[2]))
        phi = np.arctan2(2 * (q[1] * q[3] + q[2] * q[0]), (q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]))
        singularitycheck(2, theta)
    elif ("yxz" == sequence) :
        psi = np.arctan2(2 * (q[1] * q[3] + q[2] * q[0]), (q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]))
        theta = np.arcsin(2 * (q[1] * q[0] - q[2] * q[3]))
        phi = np.arctan2(2 * (q[1] * q[2] + q[3] * q[0]), (q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3]))
        singularitycheck(2, theta)
    elif ("zyx" == sequence) :
        psi = np.arctan2(2 * (q[1] * q[2] + q[3] * q[0]), (q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]))
        theta = np.arcsin(2 * (q[2] * q[0] - q[1] * q[3]))
        phi = np.arctan2(2 * (q[1] * q[0] + q[3] * q[2]), (q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]))
        singularitycheck(2, theta)
    # // matlab way:
    # // psi = np.arctan2( 2.*(q[1]*q[2] + q[0]*q[3]) , q[0]*q[0] + q[1]*q[1]- q[2]*q[2] - q[3]*q[3])
    # // theta = np.arcsin( 2.*( - q[1]*q[3] + q[0]*q[2]))
    # // phi = np.arctan2( 2.*(q[2]*q[3] + q[0]*q[1]) , q[0]*q[0] - q[1]*q[1]- q[2]*q[2] + q[3]*q[3])
    else :
        print("The sequence is not in the expected format (\"xyz\")\n")

    return(psi,theta,phi)

def Mat_fusee_terre(X):
    """
    paramètre : vecteur X = (x,y,z,vx,vy,vz,q0,q1,q2,q3,p,q,r)
    Matrice de changement de base entre le repère terrestre et le repere de la fusee
    """
    q0, q1, q2, q3 = X[6], X[7], X[8], X[9]
    return np.array(
        [
            [
                q0 ** 2 + q1 ** 2 - q2 ** 2 - q3 ** 2   , 2 * (q1 * q2 + q0 * q3)                   , 2 * (q1 * q3 - q0 * q2),
            ],
            [
                2 * (q1 * q2 - q0 * q3)                 , q0 ** 2 - q1 ** 2 + q2 ** 2 - q3 ** 2     , 2 * (q2 * q3 + q0 * q1),
            ],
            [
                2 * (q1 * q3 + q0 * q2)                 , 2 * (q2 * q3 - q0 * q1)                   , q0 ** 2 - q1 ** 2 - q2 ** 2 + q3 ** 2,
            ],
        ],
    )
def Mat_deriv_quaternion(X):
    [p,q,r] = X[-3:]
    return  np.array(
        [[0, -p, -q, -r], 
        [p,   0,  r, -q], 
        [q,  -r,  0,  p], 
        [r,   q, -p,  0]])