import paho.mqtt.client as mqtt
import json

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("data")

def on_message(client, userdata, msg):
    print(msg.topic + " " + str(msg.payload))

    data = json.loads(msg.payload)
    print(data)

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect("192.168.1.8", 1883, 60)
client.loop_forever()