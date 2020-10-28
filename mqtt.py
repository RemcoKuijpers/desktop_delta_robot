import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish
from delta_robot import DeltaRobotController
import time

class MqttClient(object):
    def __init__(self):
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect("localhost", 1883, 60)
        self.robot = DeltaRobotController()
        self.x = 0
        self.y = 0
        self.z = -200
        self.robot.sendInterpolatedCartesianMove(self.x,self.y,self.z,1)

    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code "+str(rc))
        self.client.subscribe("x")
        self.client.subscribe("y")
        self.client.subscribe("z")
        self.client.subscribe("home")
        self.client.subscribe("circle")

    def on_message(self, client, userdata, msg):
        if msg.topic == "x":
            self.x = int(msg.payload) - 50
            self.robot.sendInterpolatedCartesianMove(self.x,self.y,self.z,0.2)
        if msg.topic == "y":
            self.y = int(msg.payload) - 50
            self.robot.sendInterpolatedCartesianMove(self.x,self.y,self.z,0.2)
        if msg.topic == "z":
            self.z = -int(msg.payload)
            self.robot.sendInterpolatedCartesianMove(self.x,self.y,self.z,0.2)
        if msg.topic == "home":
            self.robot.homeServos(t=1)
        if msg.topic == "circle":
            timeout = time.time() + 5
            while True:
                self.robot.sickCircleMovements()
                if time.time() > timeout:
                    break

    def listen(self):
        self.client.loop_forever()

if __name__ == "__main__":
    mqtt = MqttClient()
    mqtt.listen()