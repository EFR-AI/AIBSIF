import numpy as np


def dichointerpo(tableau, valeur):
    """
    dichotomie adaptée pour ma fonction interpolation
    """
    gauche = 0
    droite = len(tableau) - 1
    if valeur <= tableau[gauche, 0]:
        return gauche
    elif valeur >= tableau[droite, 0]:
        return droite + 1
    while droite != gauche + 1:
        milieu = (droite + gauche) // 2
        if valeur == tableau[milieu, 0]:
            return milieu
        elif valeur < tableau[milieu, 0]:
            droite = milieu
        else:
            gauche = milieu
    return droite


def interpolation(tableau, valeur):
    """
    paramètre : un tableau à deux dimensions [[x0,y0],[x1,y1]...] et un float.
    Renvoie la valeur interpolée linéairement d'abscisse la valeur entrée.
    Si la valeur est inferieure ou supérieur au valeur extrémale des abscisses alors la fonction renvoie la valeur extrémale des ordonnées.
    """
    i = 0
    abscisse = 0
    ordonnee = 1
    i = dichointerpo(tableau, valeur)
    if i == len(tableau):
        return tableau[-1, ordonnee]
    elif i == 0:
        return tableau[0, ordonnee]
    elif tableau[i, abscisse] == valeur:
        return tableau[i, ordonnee]
    else:
        accroissement = (tableau[i, ordonnee] - tableau[i - 1, ordonnee]) / (
            tableau[i, abscisse] - tableau[i - 1, abscisse]
        )

    return tableau[i - 1, ordonnee] + accroissement * (
        valeur - tableau[i - 1, abscisse]
    )
