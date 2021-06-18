import time
import json
import paho.mqtt.client as mqtt
import numpy as np

from sensor_fusion import madgwick, kalman, complementary
from plot import plot_trajectory, fft, plot_trajectory_attenuated
import line_notifier as notifier

acc = []
gyr = []
mag = []
timestamp = []

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("data")

def on_message(client, userdata, msg):
    global acc, gyr, mag, timestamp
    data = json.loads(msg.payload)

    acc.append([float(data['acc_X']), float(data['acc_Y']), float(data['acc_Z'])])
    gyr.append([float(data['gyro_X']), float(data['gyro_Y']), float(data['gyro_Z'])])
    mag.append([float(data['mag_X']), float(data['mag_Y']), float(data['mag_Z'])])
    timestamp.append(float(data['timestamp']))

def setup_mqtt():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect('192.168.1.8', port=1883, keepalive=60)
    return client

def get_raw_data(client, duration):
    global acc, gyr, mag, timestamp

    print('Start measuring...')
    notifier.line_message('Start measuring...')
    client.loop_start()
    time.sleep(duration * 1.1)
    client.loop_stop()
    print(f'Finish measuring... Duration = {timestamp[-1] - timestamp[0]}')
    notifier.line_message('Finish measuring...')

    acc = np.array(acc)   # unit: m/s2
    gyr = np.array(gyr)   # unit: dps
    gyr = np.radians(gyr) # unit: rad/s
    mag = np.array(mag)   # unit: gauss
    mag = 0.1 * mag       # unit: mT

    return acc, gyr, mag


if __name__ == '__main__':
    client = setup_mqtt()

    DURATION = 3 # unit: s
    acc, gyr, mag = get_raw_data(client, DURATION)
    df = complementary(acc, gyr, mag, len(acc)/DURATION)
    print(df)

    dt = DURATION / len(df)

    # plot_trajectory(df, dt)
    df = fft(df, dt)
    plot_trajectory_attenuated(df, dt)