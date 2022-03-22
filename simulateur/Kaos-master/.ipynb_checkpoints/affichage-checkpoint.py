import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, ArrowStyle
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import proj3d

import numpy as np
from quaternion import quaternion_to_cardan, Mat_fusee_terre


class myArrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0, 0), (0, 0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        FancyArrowPatch.draw(self, renderer)


def format_angle_quaternion(Y, echantillonage=1):
    """
    retourne les 3 angles de cardan de la fusée au cours du temps
    """
    list_angle = np.array(
        [quaternion_to_cardan(Y[i][6:10]) for i in range(0, len(Y), echantillonage)]
    )
    list_phi = list_angle[:, 0]
    list_theta = list_angle[:, 1]
    list_psi = list_angle[:, 2]
    return list_phi, list_theta, list_psi


def format_vecteurs(Y, echantillonage, norme, axes=(1, 0, 0)):

    list_2eme_point = np.array(
        [
            np.transpose(Mat_fusee_terre(Y[i])).dot(
                np.array([axes[0], axes[1], axes[2]]) * 6 * norme / 75
            )
            for i in range(0, len(Y), echantillonage)
        ]
    )
    list_u = list_2eme_point[:, 0]
    list_v = list_2eme_point[:, 1]
    list_w = -list_2eme_point[:, 2]
    return list_u, list_v, list_w


def affichage_arrow_dynamic(list_x, list_y, list_z, list_u, list_v, list_w):
    """
    affiche d'une flèche dynamique représentant la fusée

    list_x,list_y,list_z : liste les positions des bases des vecteurs
    list_u,list_v,list_w : listes des 3 coordonnées des vecteur
    """
    global quiver
    fig = plt.figure()
    ax2 = fig.add_subplot(111, projection="3d")
    longueur = len(list_x)

    data = np.c_[np.array(list_x), np.array(list_y)]
    data = np.c_[data, np.array(list_z)]
    data = np.c_[data, np.array(list_u)]
    data = np.c_[data, np.array(list_v)]
    data = np.c_[data, np.array(list_w)]
    data = np.transpose(data)

    ax2.set_xlim3d([np.min(list_x), np.max(list_x)])
    ax2.set_xlabel("X")

    ax2.set_ylim3d([np.min(list_y), np.max(list_y)])
    ax2.set_ylabel("Y")

    ax2.set_zlim3d([np.min(list_z), np.max(list_z)])
    ax2.set_zlabel("Z")

    ax2.set_title("Vol de la fusée")

    lines = ax2.plot(data[0, 0], data[1, 0], data[2, 0], ".", markersize=1)[0]
    a = myArrow3D(
        [data[0, 0], data[3, 0]],
        [data[1, 0], data[4, 0]],
        [data[2, 0], data[5, 0]],
        mutation_scale=20,
        arrowstyle=ArrowStyle(
            "simple", head_length=0.3, head_width=0.3, tail_width=0.3
        ),
        color="dimgrey",
    )
    quiver = ax2.add_artist(a)

    def update(num, data, lines):
        global quiver
        quiver.remove()
        a = myArrow3D(
            [data[0, num], data[0, num] + data[3, num]],
            [data[1, num], data[1, num] + data[4, num]],
            [data[2, num], data[2, num] + data[5, num]],
            mutation_scale=20,
            arrowstyle=ArrowStyle(
                "simple", head_length=0.3, head_width=0.3, tail_width=0.3
            ),
            color="dimgrey",
        )
        quiver = ax2.add_artist(a)
        if num > 10:
            lines.set_data(data[0:2, 0 : (num - 10) : 15])
            lines.set_3d_properties(data[2, 0 : (num - 10) : 15])

    ani = animation.FuncAnimation(
        fig, update, longueur, fargs=(data, lines), interval=1
    )

    # pour enregistrer l'animation, attention tres long
    # ani.save('enregistrement7.mp4',fps=200)
    plt.show()


def affichage_quiver(
    first_part,
    second_part,
    ind_apogee,
    list_x,
    list_y,
    list_z,
    list_u1,
    list_v1,
    list_w1,
    list_u2,
    list_v2,
    list_w2,
    list_u3,
    list_v3,
    list_w3,
):
    """
    affichage de la trajectoire de la fusée par une suite de vecteur
    du au pas constant utilise dans la méthode d'intégration, il y a beaucoup plus de point à la descente (plus lente) qu'à la montée
    first_part : facteur d'echantillonage à la montée
    second_part : facteur d'echantillonage à la descente
    ind_apogee  : indice de l'apogée pour séparée la montée de la descente

    possibilté si on decommente certaines lignes d'afficher le repère complet (listes en ---2 et ---3)
    list_x,list_y,list_z : position du centre du vecteur
    list_u-,list_v-,list_w- : coordonnee du vecteur
    """
    fig = plt.figure()
    #fig.patch.set_facecolor('#FFFFFF')
    #fig.patch.set_alpha(0.7)

    ax2 = fig.add_subplot(111, projection="3d")

    data = np.c_[np.array(list_x), np.array(list_y)]
    data = np.c_[data, np.array(list_z)]
    data1 = np.c_[data, np.array(list_u1)]
    data1 = np.c_[data1, np.array(list_v1)]
    data1 = np.c_[data1, np.array(list_w1)]

    data2 = np.c_[data, np.array(list_u2)]
    data2 = np.c_[data2, np.array(list_v2)]
    data2 = np.c_[data2, np.array(list_w2)]

    data3 = np.c_[data, np.array(list_u3)]
    data3 = np.c_[data3, np.array(list_v3)]
    data3 = np.c_[data3, np.array(list_w3)]

    data = np.transpose(data)

    ax2.quiver(
        list_x[0:ind_apogee:first_part],
        list_y[0:ind_apogee:first_part],
        list_z[0:ind_apogee:first_part],
        list_u1[0:ind_apogee:first_part],
        list_v1[0:ind_apogee:first_part],
        list_w1[0:ind_apogee:first_part],
        pivot="middle",
        color="#00a3ca",
    )
    ax2.quiver(
        list_x[ind_apogee:-1:second_part],
        list_y[ind_apogee:-1:second_part],
        list_z[ind_apogee:-1:second_part],
        list_u1[ind_apogee:-1:second_part],
        list_v1[ind_apogee:-1:second_part],
        list_w1[ind_apogee:-1:second_part],
        pivot="middle",
        color="#00a3ca",
    )

    # ax2.quiver(list_x[0:ind_apogee:first_part], list_y[0:ind_apogee:first_part], list_z[0:ind_apogee:first_part], list_u2[0:ind_apogee:first_part], list_v2[0:ind_apogee:first_part], list_w2[0:ind_apogee:first_part], pivot='middle',color = '#2f5597')
    # ax2.quiver(list_x[ind_apogee:-1:second_part], list_y[ind_apogee:-1:second_part], list_z[ind_apogee:-1:second_part], list_u2[ind_apogee:-1:second_part], list_v2[ind_apogee:-1:second_part], list_w2[ind_apogee:-1:second_part], pivot='middle',color = '#2f5597')

    # ax2.quiver(list_x[0:ind_apogee:first_part], list_y[0:ind_apogee:first_part], list_z[0:ind_apogee:first_part], list_u3[0:ind_apogee:first_part], list_v3[0:ind_apogee:first_part], list_w3[0:ind_apogee:first_part], pivot='middle',color = '#2f5597')
    # ax2.quiver(list_x[ind_apogee:-1:second_part], list_y[ind_apogee:-1:second_part], list_z[ind_apogee:-1:second_part], list_u3[ind_apogee:-1:second_part], list_v3[ind_apogee:-1:second_part], list_w3[ind_apogee:-1:second_part], pivot='middle',color = '#2f5597')

    ax2.set_xlim3d([np.min(list_x), np.max(list_x)])
    ax2.set_xlabel("X")

    ax2.set_ylim3d([np.min(list_y), np.max(list_y)])
    ax2.set_ylabel("Y")

    ax2.set_zlim3d([np.min(list_z), np.max(list_z)])
    ax2.set_zlabel("Z")

    ax2.set_title("Vol de la fusée")
    
    #plt.show()


def vent(wind_arg,altitude, aggrandissement=1):
    """
    affiche la colonne de vent, dégradé de couleur suivant la force du vent

    altitude : hauteur de la colonne de vent
    facteur d'agrandissement si le vent est faible pour mieux le visualiser
    """
    fig = plt.figure()
    ax2 = fig.add_subplot(111, projection="3d")

    # affiche une colonne mais peut être remplacer pour afficher un champs de vecteur
    x, y, z = np.meshgrid([50], [50], np.arange(0, altitude, altitude / 60))
    champ_vent = np.array(
        [[[wind_arg.return_wind(h) for h in ligne] for ligne in carre] for carre in z]
    )

    u = champ_vent[:, :, :, 0] * aggrandissement
    v = champ_vent[:, :, :, 1] * aggrandissement
    w = champ_vent[:, :, :, 2] * aggrandissement

    list_couleur = np.sqrt(u ** 2 + v ** 2).ravel()
    list_couleur = list_couleur / list_couleur.max()
    list_couleur = np.concatenate((list_couleur, np.repeat(list_couleur, 2)))

    ax2.quiver(x, y, z, u, v, w, colors=plt.cm.rainbow(list_couleur))

    ax2.set_xlim3d([0, 100])
    ax2.set_xlabel("X")

    ax2.set_ylim3d([0, 100])
    ax2.set_ylabel("Y")

    ax2.set_zlim3d([0, altitude])
    ax2.set_zlabel("Z")

    ax2.set_title("Vent")
    #plt.show()


def affichage_angle_quaternion(list_z, list_phi, list_theta, list_psi, list_vent):
    """
    affiche l'evolution des 3 angles d'orientation au cours de l'altitude ainsi que d'un paramètre de vent pour visualiser la stabilisation
    list_z : liste d'altitude
    list_phi,list_theta,list_psi : les 3 angles de cardan pour l'orientation
    list_vent : liste des vecteurs de vent shape : (n,3)
    """
    plt.figure()
    plt.plot(list_z, list_phi % 2 * np.pi)
    plt.plot(list_z, list_theta % 2 * np.pi)
    plt.plot(list_z, list_psi % 2 * np.pi)
    plt.plot(list_z, list_vent[:, 1] * 0.1)
    plt.xlabel("temps")
    plt.ylabel("angle en radian/ norme du vent/10")
    plt.title("évolution de l'orientation de la fusée ")
    plt.legend(["phi", "theta", "psi"])
    #plt.show()


def affichage_parametre(
    x, y, label_abscisse="x", label_ordonnee="y", titre="Trajectoire", c="#2f5597"
):
    """
    affiche 2 parametres du vol l'un en fonction de l'autre
    permet de donner des étiquettes aux axes et un titre, ainsi qu'une couleur à la courbe
    """
    plt.figure()
    plt.plot(x, y, color=c)
    plt.xlabel(label_abscisse)
    plt.ylabel(label_ordonnee)
    plt.title(titre)
    #plt.show()
