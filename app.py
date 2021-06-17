from imu import gy801, tilt_compensated_heading
from math import sin, cos, atan2, degrees, radians
import time

import socket

import dash
import dash_core_components as dcc
import dash_html_components as html
import plotly.express as px
import pandas as pd

external_stylesheets = ['https://codepen.io/chriddyp/pen/bWLwgP.css']

app = dash.Dash(__name__, external_stylesheets=external_stylesheets)

# assume you have a "long-form" data frame
# see https://plotly.com/python/px-arguments/ for more options
df = pd.DataFrame({
    "Fruit": ["Apples", "Oranges", "Bananas", "Apples", "Oranges", "Bananas"],
    "Amount": [4, 1, 2, 2, 4, 5],
    "City": ["SF", "SF", "SF", "Montreal", "Montreal", "Montreal"]
})

fig = px.bar(df, x="Fruit", y="Amount", color="City", barmode="group")

app.layout = html.Div(children=[
    html.H1(children='Hello Dash'),

    html.Div(children='''
        Dash: A web application framework for Python.
    '''),

    dcc.Graph(
        id='example-graph',
        figure=fig
    )
])

if __name__ == '__main__':
    # app.run_server(debug=True, host='192.168.0.19')


    try:
        while True:
            try:
                sensors = gy801()
                break
            except OSError:
                print('OSError...')
                time.sleep(1)
                continue


        accel = sensors.accel
        gyro = sensors.gyro
        compass = sensors.compass

        # pitch_pre = accel.getPitch()
        # roll_pre = accel.getRoll()
        
        while True:
            # try:
            #     pitch_cur = (pitch_pre + gyro.getXangle()) * 0.98 + adxl345.getPitch() * 0.02
            #     roll_cur = (roll_pre + gyro.getYangle()) * 0.98 + adxl345.getRoll() * 0.02
            # except OSError:
            #     print('Skip once')
            #     time.sleep(0.5)
            #     continue

            # print ("ACC: ")
            # print ("x = %.3f m/s2" % ( adxl345.getX() ))
            # print ("y = %.3f m/s2" % ( adxl345.getY() ))
            # print ("z = %.3f m/s2" % ( adxl345.getZ() ))
            # print()
            # print ("Gyro: ")
            # print ("Xangle = %.3f deg" % ( gyro.getXangle() ))
            # print ("Yangle = %.3f deg" % ( gyro.getYangle() ))
            # print ("Zangle = %.3f deg" % ( gyro.getZangle() ))
            # print()
            # print(f"Pitch = {pitch_cur}")
            # print(f"Roll = {roll_cur}")
            # print()
            # print(f"Pitch = {adxl345.getPitch()}")
            # print(f"Roll = {adxl345.getRoll()}")

            try:
                magx = compass.getX()
                magy = compass.getY()
                magz = compass.getZ()

                pitch = accel.getPitch()
                roll = accel.getRoll()
            except OSError:
                print('Skip once...')
                time.sleep(0.5)
                continue

            compx = magx * cos(radians(pitch)) + magz * sin(radians(pitch))
            compy = magx * sin(radians(roll)) * sin(radians(pitch)) + \
                    magy * cos(radians(roll)) - \
                    magz * sin(radians(roll)) * cos(radians(pitch))

            bearing1 = degrees(atan2(magy, magx))
            if (bearing1 < 0):
                bearing1 += 360
            if (bearing1 > 360):
                bearing1 -= 360
            bearing1 = bearing1 + compass.angle_offset

            bearing2  = degrees(atan2(compy, compx))
            if (bearing2 < 0):
                bearing2 += 360
            if (bearing2 > 360):
                bearing2 -= 360
            bearing2 = bearing2 + compass.angle_offset

            # print ("X = %d ," % ( magx ))
            # print ("Y = %d ," % ( magy ))
            # print ("Z = %d (gauss)" % ( magz ))
            print ("Pitch = %.3f" % ( pitch ))
            print ("Roll = %.3f" % ( roll ))
            print ("tiltX = %.3f" % ( compx ))
            print ("tiltY = %.3f" % ( compy ))
            print ("Angle offset = %.3f deg" % ( compass.angle_offset ))
            print ("Original Heading = %.3f deg, " % ( bearing1 )) 
            print ("Tilt Heading = %.3f deg, " % ( bearing2 ))
            # print ("Heading = %.3f" % (tilt_compensated_heading(compy, compx))) 
            print('===================================')
            print()

            # pitch_pre = pitch_cur
            # roll_pre = roll_cur

            time.sleep(1)
            
    except KeyboardInterrupt:
        print("Cleanup")