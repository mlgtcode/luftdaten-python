#!/usr/bin/env python3

import argparse
import datetime
import time
import toml
import requests
import json
import numpy as np
import board
import busio
import subprocess
import Adafruit_DHT
import adafruit_bme280
import adafruit_bme280.advanced as adafruit_bme280
from paho.mqtt import client as mqtt_client
from sds011lib import SDS011QueryReader


# Parse command line args
parser = argparse.ArgumentParser(description='Luftdaten in Python for Raspberry Pi')
parser.add_argument('-c', '--config', default='config.toml', help='path to config file')
args = parser.parse_args()

# Read config file
config = toml.load(args.config)

# Configure Logging
import logging
logging.basicConfig(level=logging.DEBUG)

# Configure BME280
# Set address if neccesary
print("initialize BME280")
i2c    = busio.I2C(board.SCL, board.SDA)
bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=0x76)

# Configure SDS011
print("initialize SDS011")
dusty = SDS011QueryReader('/dev/ttyUSB0')

# Configure DHT22
print("initialize DHT22")
dht22 = Adafruit_DHT.DHT22
# Set pin for DHT22 is neccesary
sensor_pin = 4

# Configure MQTT
mqtt_conn = None
mqtt_cfg  = config["mqtt"]
if mqtt_cfg["enabled"]:
    mqtt_conn = mqtt_client.Client(mqtt_cfg["client_id"])
    mqtt_conn.connect(mqtt_cfg["broker"], mqtt_cfg["port"])

class Measurement:
    def __init__(self):
        self.pm25_value  = None
        self.pm10_value  = None

        if dusty:
            pm25_values = []
            pm10_values = []
            dusty.wake()
            time.sleep(5) #sensor warmup
            #dusty.set_working_period(0)
            try:
                for a in range(8):
                    values = dusty.query()
                    if values is not None:
                        pm10_values.append(values.pm10)
                        pm25_values.append(values.pm25)
            except:
                    print("Error while loading dust sensor data.")

            try:
                    dusty.sleep()
            except:
                    print("Error setting sensor to sleep. The sensor may still be in sleep mode now (bug!).")

            self.pm25_value  = np.mean(pm25_values)
            self.pm10_value  = np.mean(pm10_values)

        self.temperature = bme280.temperature
        self.humidity    = bme280.relative_humidity
        self.pressure    = bme280.pressure

        self.dhthumidity, self.dhttemperature = Adafruit_DHT.read_retry(dht22, sensor_pin)

        self.signal = wifi_signal()

    def sendMQTT(self):
        mqtt_conn.publish(mqtt_cfg["topic"], json.dumps({
            "dust_pm10":  self.pm10_value,
            "dust_pm25":  self.pm25_value,
            "temperature": self.temperature,
            "pressure":    self.pressure,
            "humidity":    self.humidity,
            "temperature_dht22": self.dhttemperature,
            "humidity_dht22":    self.dhthumidity,
        }))

    def push2opensense(self):
        cfg = config['opensense']

        if not cfg['enabled']:
            return

        senseBox_ID = "000000000000000000000" # your senseBox ID
        pm10_ID = "000000000000000000000" # your senseBox PM10 sensor ID
        pm25_ID = "000000000000000000000" # your senseBox PM25 sensor ID
        temperature_ID = "000000000000000000000" # your senseBox temperature sensor ID
        humidity_ID = "000000000000000000000" # your senseBox humidity sensor ID
        pressure_ID = "000000000000000000000" # your senseBox pressure sensor ID
        temperature2_ID = "000000000000000000000" # your senseBox temperature sensor ID
        humidity2_ID = "000000000000000000000" # your senseBox humidity sensor ID
        wifi_ID = "000000000000000000000" # wifi signal
        ts = datetime.datetime.utcnow().isoformat("T")+"Z" # RFC 3339 Timestamp 

        try:
            requests.post("https://api.opensensemap.org/boxes/"+senseBox_ID+"/data",
                json={
                    # SDS011 P10
                    pm10_ID: [round(self.pm10_value, 2), ts],
                    # SDS011 P25
                    pm25_ID: [round(self.pm25_value, 2), ts],
                    # BME Temp
                    temperature_ID: [round(self.temperature, 2), ts],
                    # BME Hum
                    humidity_ID: [round(self.humidity, 2), ts],
                    # BME Press
                    pressure_ID: [round(self.pressure/100, 2), ts], # Pressure in hPA, remove "\100" if you want the value in Pa
                    # DHT Temp
                    temperature2_ID: [round(self.dhttemperature, 2), ts],
                    # DHIT Hum
                    humidity2_ID: [round(self.dhthumidity, 2), ts],
                    # WiFi signal
                    wifi_ID: [self.signal, ts],
                },
                headers={
                    "content-type": "application/json",
                    "Authorization": "YOUR_API_KEY",
                }, # add your own api key
                timeout=20 # optional
            )
        except:
            pass

    def sendInflux(self):
        cfg = config['influxdb']

        if not cfg['enabled']:
            return

        data = "feinstaub,node={} SDS_P1={:0.2f},SDS_P2={:0.2f},BME280_temperature={:0.2f},BME280_pressure={:0.2f},BME280_humidity={:0.2f},DHT22_temperature={:0.2f},DHT22_humidity={:0.2f},Signal={:0.2f}".format(
            cfg['node'],
            self.pm10_value,
            self.pm25_value,
            self.temperature,
            self.pressure,
            self.humidity,
            self.dhttemperature,
            self.dhthumidity,
            self.signal,
        )

        requests.post(cfg['url'],
            auth=(cfg['username'], cfg['password']),
            data=data,
        )

    def sendLuftdaten(self):
        if not config['luftdaten']['enabled']:
            return

        self.__pushLuftdaten('https://api-rrd.madavi.de/data.php', 0, {
            "SDS_P1":             self.pm10_value,
            "SDS_P2":             self.pm25_value,
            "BME280_temperature": self.temperature,
            "BME280_pressure":    self.pressure,
            "BME280_humidity":    self.humidity,
            "DHT22_temperature":  self.dhttemperature,
            "DHT22_humidity":     self.dhthumidity,
            "wifi-signal":        self.signal,
        })
        
        self.__pushLuftdaten('https://api.luftdaten.info/v1/push-sensor-data/', 1, {
            "P1": self.pm10_value,
            "P2": self.pm25_value,
        })
        
        self.__pushLuftdaten('https://api.luftdaten.info/v1/push-sensor-data/', 11, {
            "temperature": self.temperature,
            "humidity":    self.humidity,
            "pressure":    round(self.pressure/100, 2),
        })

        self.__pushLuftdaten('https://api.luftdaten.info/v1/push-sensor-data/', 7, {
            "temperature": self.dhttemperature,
            "humidity":    self.dhthumidity,
        })

    def __pushLuftdaten(self, url, pin, values):
        requests.post(url,
            json={
                "software_version": "python-sds011lib",
                "sensordatavalues": [{"value_type": key, "value": val} for key, val in values.items()],
            },
            headers={
                "X-PIN":    str(pin),
                "X-Sensor": sensorID,
            }
        )

# extracts serial from cpuinfo
def getSerial():
    with open('/proc/cpuinfo','r') as f:
        for line in f:
            if line[0:6]=='Serial':
                return(line[10:26])
    raise Exception('CPU serial not found')

# get wifi signal
def wifi_signal():
    process = subprocess.Popen("iwconfig wlan0", stdout=subprocess.PIPE, shell=True)
    output, error = process.communicate()

    for line in output.split(b'\n'):
        if b"Signal level" in line:
            wifi_dbm = int(line.split(b'=')[2].split(b' ')[0])
            return wifi_dbm

def run():
    m = Measurement()

    print('pm2.5     = {:f} '.format(m.pm25_value))
    print('pm10      = {:f} '.format(m.pm10_value))
    print('Temp      = {:0.2f} deg C'.format(m.temperature))
    print('Humidity  = {:0.2f} %'.format(m.humidity))
    print('Pressure  = {:0.2f} hPa'.format(m.pressure/100))
    print('DHT Temp  = {0:0.1f} deg C'.format(m.dhttemperature))
    print('DHT Hum   = {0:0.1f}%'.format(m.dhthumidity))
    print('Serial    = ' + getSerial())
    print('Signal    = {:0.2f} '.format(m.signal))

    if mqtt_conn:
        m.sendMQTT()

    m.sendLuftdaten()
    m.push2opensense()
    # m.sendInflux()


sensorID  = config['luftdaten'].get('sensor') or ("raspi-" + getSerial())
starttime = time.time()

if __name__ == "__main__":
    while True:
        print("running ...")
        run()
        print("sleeping ...")
        # time.sleep(120)
        time.sleep(60.0 - ((time.time() - starttime) % 60.0))
    print("Stopped")
