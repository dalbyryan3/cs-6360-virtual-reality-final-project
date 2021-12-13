# %%
import matplotlib.pyplot as plt
import pandas as pd
import random

# %%
data_index_filepath = './vr_handheld_controller_gesture_data.csv'
data_index = pd.read_csv('{0}'.format(data_index_filepath), header=None)

random_data_sample = data_index.iloc[random.randint(0, data_index.shape[0]-1),:].to_numpy()

filepath = random_data_sample[0]
label = random_data_sample[1]
dt = random_data_sample[2]

data = pd.read_csv(filepath, header=None).iloc[:,:-1].to_numpy() # Note remove last column which is nan since csv ends with , at each row


plt.figure()
plt.plot(data[0], label='AccX')
plt.plot(data[1], label='AccY')
plt.plot(data[2], label='AccZ')
plt.title('Training Accelerometer Data, label={0}, dt={1}s'.format(label,dt))
plt.xlabel('Sample Number (Roughly corresponds to time)')
plt.ylabel('Accelerometer Reading in g')
plt.legend()
plt.show()

plt.plot(data[3], label='GyrX')
plt.plot(data[4], label='GyrY')
plt.plot(data[5], label='GryZ')
plt.title('Training Gyroscope Data, label={0}, dt={1}s'.format(label,dt))
plt.xlabel('Sample Number (Roughly corresponds to time)')
plt.ylabel('Gyroscope Readings in deg/s')
plt.legend()
plt.show()

plt.plot(data[6], label='GyrX')
plt.plot(data[7], label='GyrY')
plt.plot(data[8], label='GryZ')
plt.title('Training Magnetometer Data, label={0}, dt={1}s'.format(label,dt))
plt.xlabel('Sample Number (Roughly corresponds to time)')
plt.ylabel('Magnetometer Readings in muT')
plt.legend()
plt.show()

# %%
