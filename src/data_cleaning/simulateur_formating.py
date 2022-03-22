import pandas as pd
from os import listdir
from os.path import isfile, join

path_res = '../../data/resultats/'
path_sens = '../../data/sensors_data/'
resname = [join(path_res, f) for f in listdir(path_res) if isfile(join(path_res, f))]
sensname = [join(path_sens, f) for f in listdir(path_sens) if isfile(join(path_sens, f))]

for res, sens in zip(resname,sensname):
    resultat = pd.read_csv(res)
    sensor = pd.read_csv(sens)
    
    resultat = resultat.iloc[:, :-1]
    resultat = resultat.astype(float)
    resultat = resultat.set_index('Temps')
    
    sensor = sensor.iloc[:, :-1]
    sensor = sensor.astype(float)
    sensor = sensor.set_index('Temps')
    
    df = resultat.join(sensor)
    
    final = res[-6:]
    
    df.to_csv('../../data/final/final'+final)