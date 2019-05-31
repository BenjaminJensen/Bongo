import paho.mqtt.client as mqttClient
import time
from influxdb import InfluxDBClient
import datetime 

def on_connect(client, userdata, flags, rc):
 
    if rc == 0:
 
        print("Connected to broker")
 
        global Connected                #Use global variable
        Connected = True                #Signal connection 
 
    else:
 
        print("Connection failed")
 
def on_message(client, userdata, message):
    json_body = [
        {
            "measurement": "bme680_calib",
            "tags": {
                "sensor": message.topic,
            },
            "time": str(datetime.datetime.now()), #"2009-11-10T23:00:00Z",
            "fields": {
                "value": float(message.payload)
            }
        }
    ]
    print(json_body)
    influx_client.write_points(json_body)

Connected = False   #global variable for the state of the connection
 
broker_address= "192.168.1.70"  #Broker address
port = 1883                         #Broker port
user = "yourUser"                    #Connection username
password = "yourPassword"            #Connection password
 
client = mqttClient.Client("Python")               #create new instance
#client.username_pw_set(user, password=password)    #set username and password
client.on_connect= on_connect                      #attach function to callback
client.on_message= on_message                      #attach function to callback
 
client.connect(broker_address, port=port)          #connect to broker
 
client.loop_start()        #start the loop

# Influxdb setup
influx_client = InfluxDBClient('192.168.1.70', 8086, 'python', 'password123', 'sensorcalibration')


# MQTT loop thread start
while Connected != True:    #Wait for connection
    time.sleep(0.1)
 
client.subscribe("outback/temperature")
client.subscribe("outback/pressure")
client.subscribe("outback/humidity")
client.subscribe("outback/gas_resistant")
 
try:
    while True:
        time.sleep(1)
 
except KeyboardInterrupt:
    print("exiting")
    client.disconnect()
    client.loop_stop()