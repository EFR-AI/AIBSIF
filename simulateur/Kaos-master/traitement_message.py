import numpy as np

def until_first_backslashx00(string):
    """ Function that take a string containing \x00 and return all the
    string until the first \x00 found.
    
    Takes : 128\\x005\\x00
    Returns : 128"""

    return string.split('\x00', 1)[0]

def encodage_mesure(i,X,Parameters):
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
                
            ]=Parameters
    Parameters = [
        dm,
        Poussee,
        Cx,
        Cn_alpha,
        Coeff,
        Distance,
        Surface,
        rho,
        Inertie,
        puissance_vent,
        Vect_vent,
        deltr,
        deltq
    ]
    C_l0, C_lp, C_mq, C_nr = Coeff
    [l_d,l_ref,l_p,l_o_xcg] = Distance

    message='['+str(i)+'];['+','.join(str(x) for x in X)
    message += '];['
    message += ','.join(str(x) for x in Parameters[:4])
    message += ','
    message += ','.join(str(C) for C in Coeff)
    message += ','
    message += ','.join(str(l) for l in Distance)
    message += ','
    message += ','.join(str(x) for x in Parameters[6:8])
    message += ','
    message += ','.join(','.join(str(x) for x in j) for j in Inertie)
    message += ',' + str(Parameters[9])
    message += ','
    message += ','.join(str(v) for v in Vect_vent)
    message += ','
    message += ','.join(str(x) for x in Parameters[11:])
    message += ']'
    return(message)

def decodage_mesure(message):
    liste=message.split(';')
    if liste[0].strip('[]')=='':
        return 0
    i=int(liste[0].strip('[]'))
    X=[float(x) for x in liste[1].strip('[]').split(',')]

    Param = liste[2].strip('()[]').split(',')
    
    dm = float(Param[0])
    Poussee = float(Param[1])
    Cx = float(Param[2])
    Cn_alpha = float(Param[3])
    Coeff = [float(c) for c in Param[4:8]]
    Distance = [float(l) for l in Param[8:12]]
    Surface = float(Param[12])
    rho = float(Param[13])
    Inertie = np.array([[float(Param[14+x+3*j]) for x in range(3)] for j in range(3)])
    puissance_vent = float(Param[23])
    Vect_vent = [float(v) for v in Param[24:27]]
    deltr = float(Param[27])
    deltq = float(Param[28])

    Parameters=[
        0,
        0,
        dm,
        Poussee,
        0,
        Cx,
        Cn_alpha,
        Coeff,
        Distance,
        Surface,
        rho,
        Inertie,
        0,
        puissance_vent,
        Vect_vent,
        deltr,
        deltq
        ]
    return (i,X,Parameters)

def encodage_commande(answer, correction):
    if correction :
        commande= str(answer)+','+str(correction[0])+','+str(correction[1])
    else:
        commande = str(answer)+',0.0,0.0'
    return commande

def decodage_commande(commande):
    liste = commande.split(',')
    correction=[float(liste[1]),float(liste[2])]
    answer = (liste[0]=='True')
    return answer, correction
