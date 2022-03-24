import pandas as pd
import numpy as np
from sklearn.utils import shuffle

def define_dataset(final_csv, number_imu):
    """
    Utilise un csv pour créer deux numpy array.
    Le premier contenant les valeurs mesurées par la ou les IMU en fonction du nombre indiqué en argument (max 3, pour l'instant).
    Le second contenant les grounds truths d'accelerations et de vitesses angulaires correspondantes. 
    """
    
    df = pd.read_csv(final_csv)
    
    real = df[[ 'Acceleration en x avec g', 'Acceleration en y avec g',
       'Acceleration en z avec g','Angle phi', 'Angle theta', 'Angle psi']]
    
    list_feature=[]
    
    for i in range(1, number_imu+1):
        list_feature.append('Accelerometre {} x (en m/s2)'.format(i))
        list_feature.append('Accelerometre {} y (en m/s2)'.format(i))
        list_feature.append('Accelerometre {} z (en m/s2)'.format(i))
        list_feature.append('Gyroscope {} phi'.format(i))
        list_feature.append('Gyroscope {} theta'.format(i))
        list_feature.append('Gyroscope {} psi'.format(i))

    mes = df[list_feature]

    return mes.values, real.values

def sequence(x,y,window_size,stride):
    """
    A partir de numpy array contenant les mesures et les ground truth
    Créer des séquences de mesure taille window_size en les espaçant de la valeur de stride.
    Puis récupère la ground truth qui se situe au moment du stride.
    Renvoie une liste de séquences de mesures avec les ground truth associées.
    """
    x_total = []
    y_total = []

    for idx in range(0, x.shape[0] - window_size - 1, stride):
        x_total.append(x[idx+1 : idx + 1 + window_size, :])
        y_total.append(y[idx + window_size//2 + stride//2, :])
    
    x_total = np.reshape(x_total, (len(x_total), x_total[0].shape[0], x_total[0].shape[1]))
    y_total = np.reshape(y_total, (len(y_total), y_total[0].shape[0]))
    
    return x_total, y_total

def build_dataset(start_seq, end_seq, window_size,stride, number_imu):
    """
    Récupère les csv contenus entre start_seq et end_seq.
    Puis en extrait les séquences (voir fonction sequence).
    Enfin, crée deux array numpy avec les listes de séquences de tout les csv choisis ainsi que les grounds truths associées.
    """
    
    final_x = []
    final_y = []
    
    for i in range(start_seq, end_seq):
        x,y = define_dataset('../../data/final_3IMU/final_{}.csv'.format(i),number_imu)
        cur_x, cur_y = sequence(x,y, window_size,stride)
        
        final_x.append(cur_x)
        final_y.append(cur_y)
    
    final_x = np.vstack(final_x)
    final_y = np.vstack(final_y)
    
    final_x, final_y = shuffle(final_x, final_y)
    
    return final_x, final_y
        