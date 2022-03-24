import pandas as pd
from os import listdir
from os.path import isfile, join
from sklearn.preprocessing import MinMaxScaler

path_res = '../../data/resultats_3IMU/'
path_sens = '../../data/sensors_data_3IMU/'
resname = [join(path_res, f) for f in listdir(path_res) if isfile(join(path_res, f))]
sensname = [join(path_sens, f) for f in listdir(path_sens) if isfile(join(path_sens, f))]
final = 0

for res, sens in zip(resname,sensname):
    resultat = pd.read_csv(res)
    sensor = pd.read_csv(sens)
    
    resultat = resultat.iloc[:, :-1]
    resultat.drop('Position en z', axis=1, inplace=True)
    resultat = resultat.astype(float)
    resultat = resultat.set_index('Temps')
    
    sensor = sensor.iloc[:, :-1]
    sensor = sensor.astype(float)
    sensor = sensor.set_index('Temps')
    
    df = resultat.join(sensor)
    
    for x in df.columns:
        scaler = MinMaxScaler()
        df[[x]] = scaler.fit_transform(df[[x]])
            
    if not df.isnull().values.any().any():
        df.to_csv('../../data/final_3IMU/final_'+str(final)+'.csv')
        final = final+1
