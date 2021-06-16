import numpy as np
from scipy.integrate import cumtrapz
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

def plot_trajectory(df, dt):
    # Double integrate accelerations to find positions
    x = cumtrapz(cumtrapz(df['EARTH LINEAR ACCELERATION X'], dx=dt), dx=dt)
    y = cumtrapz(cumtrapz(df['EARTH LINEAR ACCELERATION Y'], dx=dt), dx=dt)
    z = cumtrapz(cumtrapz(df['EARTH LINEAR ACCELERATION Z'], dx=dt), dx=dt)
    # Plot 3D Trajectory
    fig, ax = plt.subplots()
    fig.suptitle('3D Trajectory of IMU', fontsize=20)
    ax = plt.axes(projection='3d')
    ax.plot3D(x, y, z, c='red', lw=5, label='IMU trajectory')
    ax.set_xlabel('X position (m)')
    ax.set_ylabel('Y position (m)')
    ax.set_zlabel('Z position (m)')
    plt.show()

def plot_trajectory_attenuated(df, dt):
    # Double integrate accelerations to calculate coordinate positions
    x = cumtrapz(cumtrapz(df['x_ifft'], dx=dt), dx=dt)
    y = cumtrapz(cumtrapz(df['y_ifft'], dx=dt), dx=dt)
    z = cumtrapz(cumtrapz(df['z_ifft'], dx=dt), dx=dt)
    # Plot attenuated 3D Trajectory
    fig, ax = plt.subplots()
    fig.suptitle('3D Trajectory of IMU', fontsize=20)
    ax = plt.axes(projection='3d')
    ax.plot3D(x, y, z, c='red', lw=5, label='Attenuated IMU trajectory')
    ax.set_xlabel('X position (m)')
    ax.set_ylabel('Y position (m)')
    ax.set_zlabel('Z position (m)')
    ax.legend(fontsize='x-large')
    plt.show()

def fft(df, dt):
    # Discrete Fourier Transform sample frequencies
    freq = np.fft.rfftfreq(df['EARTH LINEAR ACCELERATION X'].size, d=dt)
    # Compute the Fast Fourier Transform (FFT) of acceleration signals
    fft_x = np.fft.rfft(df['EARTH LINEAR ACCELERATION X']) 
    fft_y = np.fft.rfft(df['EARTH LINEAR ACCELERATION Y']) 
    fft_z = np.fft.rfft(df['EARTH LINEAR ACCELERATION Z'])
    # Plot Frequency spectrum
    fig, [ax1, ax2, ax3] = plt.subplots(3, 1, sharex=True, sharey=True)

    for f, fx, fy, fz in zip(freq, abs(fft_x), abs(fft_y), abs(fft_z)):
        print(f'freq = {f:.2f} ({fx:.3f}, {fy:.3f}, {fz:.3f})')

    fig.suptitle('Noise Spectrum',fontsize=20)
    ax1.plot(freq, abs(fft_x), c='r', label='x noise')
    ax1.legend()
    ax2.plot(freq, abs(fft_y), c='b', label='y noise')
    ax2.legend()
    ax3.plot(freq, abs(fft_z), c='g', label='z noise')
    ax3.legend()
    ax3.set_xlabel('Freqeuncy (Hz)')
    plt.show()

    # Attenuate noise in X, Y, Z below 0.1Hz and above 6Hz by 10 dB
    atten_x_fft = np.where((freq > 6) | (freq < 0.1), fft_x * 0.1, fft_x) 
    atten_y_fft = np.where((freq > 6) | (freq < 0.1), fft_y * 0.1, fft_y) 
    atten_z_fft = np.where((freq > 6) | (freq < 0.1), fft_z * 0.1, fft_z)
    # Compute inverse of discrete Fourier Transform 
    df['x_ifft'] = np.fft.irfft(atten_x_fft, n=df.shape[0])
    df['y_ifft'] = np.fft.irfft(atten_y_fft, n=df.shape[0])
    df['z_ifft'] = np.fft.irfft(atten_z_fft, n=df.shape[0])
    # Plot new acceleration signals
    cols_raw = ['EARTH LINEAR ACCELERATION X','EARTH LINEAR ACCELERATION Y','EARTH LINEAR ACCELERATION Z']
    cols_new = ['x_ifft','y_ifft','z_ifft']

    plt.subplot(1, 3, 1)
    plt.plot(df['EARTH LINEAR ACCELERATION X'])
    plt.plot(df['x_ifft'], label='x_ifft')
    plt.legend()

    plt.subplot(1, 3, 2)
    plt.plot(df['EARTH LINEAR ACCELERATION Y'])
    plt.plot(df['y_ifft'], label='y_ifft')
    plt.legend()

    plt.subplot(1, 3, 3)
    plt.plot(df['EARTH LINEAR ACCELERATION Z'])
    plt.plot(df['z_ifft'], label='z_ifft')
    plt.legend()
    
    plt.show()

    return df