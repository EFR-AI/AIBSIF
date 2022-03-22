import pandas as pd 
import numpy as np
import tensorflow as tf
from window_generator import WindowGenerator
from os import listdir
from os.path import isfile, join

def create_dataset(window_size=6, train_size=0.8, val_size=0.1, test_size=0.1) :

    assert(train_size + val_size + test_size == 1)
    assert(window_size >= 2)

    path = '../../data/final/'
    filanemes = [join(path, f) for f in listdir(path) if isfile(join(path, f))]

    for i, file in enumerate(filanemes) :

        df = pd.read_csv(path + file)

        df.set_index('Temps', inplace=True)
        df.dropna(inplace=True)

        n = len(df)
        train_df = df.iloc[0:int(n*train_size)]
        val_df = df.iloc[int(n*train_size):int(n*(train_size+val_size))]
        test_df = df.iloc[int(n*(train_size+val_size)):]
        
        label = ['Acceleration en x avec g', 'Acceleration en y avec g',
       'Acceleration en z avec g', 'Angle phi', 'Angle theta', 'Angle psi']
        new = WindowGenerator(window_size, 1, 1, train_df, val_df, test_df, label)

        if i != 0 :
            train = train.concatenate(new.train)
            val = val.concatenate(new.val)
            test = test.concatenate(new.test)
        
        else :
            train = new.train
            val = new.val
            test = new.test
    
    return train, val, test