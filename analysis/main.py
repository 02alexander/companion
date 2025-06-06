#!/usr/bin/env python3

import rerun as rr
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib

# matplotlib.use('Qt6Agg')
matplotlib.use('TkAgg')

import rerun as rr

path_to_rrd = "./step_response.rrd"

recording = rr.dataframe.load_recording(path_to_rrd)
batches = recording.view(index="sample_time", contents="/**").select()
df: pd.DataFrame = batches.read_pandas()


print(df.columns)


Fs = 100

ts = np.array(df['sample_time']) / 1e9
# ts = np.array(df[df['sample_time'] > pd.to_timedelta(1e9*1) ]) / 1e9
control_signal = np.array(df['/control:Scalar'])
velocity = np.array(df['/velocity:Scalar'])

# plt.plot(ts, velocity)
# plt.show()

