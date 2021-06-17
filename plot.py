import numpy as np
from scipy.integrate import cumtrapz
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from math import degrees, atan2, asin, sin, cos
import line_notifier as notifier
from sensor_fusion import R_x, R_y, R_z

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

    plt.savefig('trajectory.png')
    notifier.send_pic("IMU trajectory: ", "trajectory.png")
    # plt.show()

def plot_trajectory_attenuated(df, dt):
    # Double integrate accelerations to calculate coordinate positions
    x = cumtrapz(cumtrapz(df['x_ifft'], dx=dt), dx=dt)
    y = cumtrapz(cumtrapz(df['y_ifft'], dx=dt), dx=dt)
    z = cumtrapz(cumtrapz(df['z_ifft'], dx=dt), dx=dt)
    # Plot attenuated 3D Trajectory
    # fig, ax = plt.subplots()
    # fig.suptitle('3D Trajectory of IMU', fontsize=20)
    # ax = plt.axes(projection='3d')
    # ax.plot3D(x, y, z, c='red', lw=5, label='Attenuated IMU trajectory')
    # ax.set_xlabel('X position (m)')
    # ax.set_ylabel('Y position (m)')
    # ax.set_zlabel('Z position (m)')
    # # ax.legend(fontsize='x-large')

    # plt.savefig('attenuated_trajectory.png')
    # notifier.send_pic("Attenuated IMU trajectory: ", "attenuated_trajectory.png")
    # plt.show()

    # Add XYZ axis arrows to indicate phone pose
    # Earth 3 axis unit vectors
    earth_x = np.array([[1,0,0],] * len(x)).T
    earth_y = np.array([[0,1,0],] * len(x)).T
    earth_z = np.array([[0,0,1],] * len(x)).T
    # Initialize body Vectors
    body_x = np.empty(earth_x.shape)
    body_y = np.empty(earth_y.shape)
    body_z = np.empty(earth_z.shape)
    # Perform inverse frame transformations (body frame <-- earth frame) 
    # body_vectors = (RxRyRz)(earth_vectors)
    roll = df['ROLL'] 
    pitch = df['PITCH'] 
    yaw = df['YAW']
    for i in range(x.shape[0]):
        # use negative angles to reverse rotation
        body_x[:,i] = R_x(-roll[i]) @ R_y(-pitch[i]) @ R_z(-yaw[i]) @earth_x[:,i]
        body_y[:,i] = R_x(-roll[i]) @ R_y(-pitch[i]) @ R_z(-yaw[i]) @ earth_y[:,i]
        body_z[:,i] = R_x(-roll[i]) @ R_y(-pitch[i]) @ R_z(-yaw[i]) @ earth_z[:,i]

    # Set length of quiver arrows    
    distance = np.sqrt(x[-1]**2 + y[-1]**2 + z[-1]**2)
    length = 0.05 * distance
    # Plot x vectors
    # downsampling to every 10th arrow ([::10])
    fig6, ax4 = plt.subplots()
    fig6.suptitle('IMU trajectory and pose',fontsize=20)
    ax4 = plt.axes(projection='3d')
    ax4.plot3D(x, y, z, c='red', lw=5, label='Attenuated IMU trajectory')
    # plot x vectors
    # ax4.quiver(x[::10],y[::10],z[::10],
    #         body_x[0][::10],body_x[1][::10],body_x[2][::10],
    #         color='b',label = 'x axis',length = length)
    # Plot y vectors
    # ax4.quiver(x[::10],y[::10],z[::10],
    #         body_y[0][::10],body_y[1][::10],body_y[2][::10],
    #         color='r',label = 'y axis',length = length)
    # plot z vectors
    ax4.quiver(x[::5],y[::5],z[::5],
            body_z[0][::5],body_z[1][::5],body_z[2][::5],
            color='g',label = 'z axis',length = length)
    ax4.set_xlabel('X position (m)')
    ax4.set_ylabel('Y position (m)')
    ax4.set_zlabel('Z position (m)')
    # ax4.set_xlim(-0.5,0.5)# may need to vary
    # ax4.set_ylim(-0.5,0.5)
    # ax4.set_zlim(-0.5,0.5)
    # ax4.legend(fontsize='x-large')
    
    plt.savefig('attenuated_trajectory_z_axiz.png')
    notifier.send_pic("Attenuated IMU trajectory with Z axis: ", "attenuated_trajectory_z_axiz.png")
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

    fig.suptitle('Frequency Spectrum',fontsize=20)
    ax1.plot(freq, abs(fft_x), c='r', label='x frequency')
    ax1.legend()
    ax2.plot(freq, abs(fft_y), c='b', label='y frequency')
    ax2.legend()
    ax3.plot(freq, abs(fft_z), c='g', label='z frequency')
    ax3.legend()
    ax3.set_xlabel('Freqeuncy (Hz)')

    plt.savefig('frequency_spectrum.png')
    notifier.send_pic("Frequency Spectrum: ", "frequency_spectrum.png")

    # Attenuate noise in X, Y, Z below 0.1Hz and above 6Hz by 10 dB
    atten_x_fft = np.where((freq > 6) | (freq < 0.1), fft_x * 0.1, fft_x) 
    atten_y_fft = np.where((freq > 6) | (freq < 0.1), fft_y * 0.1, fft_y) 
    atten_z_fft = np.where((freq > 6) | (freq < 0.1), fft_z * 0.1, fft_z)
    # Compute inverse of discrete Fourier Transform 
    df['x_ifft'] = np.fft.irfft(atten_x_fft, n=df.shape[0])
    df['y_ifft'] = np.fft.irfft(atten_y_fft, n=df.shape[0])
    df['z_ifft'] = np.fft.irfft(atten_z_fft, n=df.shape[0])

    plt.subplot(1, 3, 1)
    plt.plot(df['EARTH LINEAR ACCELERATION X'], label='Before')
    plt.plot(df['x_ifft'], label='After')
    plt.legend()

    plt.subplot(1, 3, 2)
    plt.plot(df['EARTH LINEAR ACCELERATION Y'], label='Before')
    plt.plot(df['y_ifft'], label='After')
    plt.legend()

    plt.subplot(1, 3, 3)
    plt.plot(df['EARTH LINEAR ACCELERATION Z'], label='Before')
    plt.plot(df['z_ifft'], label='After')
    plt.legend()
    
    plt.savefig('ifft.png')
    notifier.send_pic("IFFT: ", "ifft.png")

    return df