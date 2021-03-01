import pandas as pd
import numpy as np

def get_control_signals(trajectory_num):
    data = pd.read_csv('trajectory - Paperbot.csv',usecols=['L'+str(trajectory_num),'R'+str(trajectory_num)])
    data = data.to_numpy()*np.pi/180
    control_signals = np.zeros((10*1000,2))
    for i in range(20):
        v_init = data[i]
        v_end = data[i+1]
        for j in range(500):
            v = v_init + (v_end - v_init)*j/500
            control_signals[i*500+j] = v
    return control_signals