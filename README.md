# dusty-python

Python 3 program for the [sensor.community](sensor.community) sensor network.
It has been written to run on a Raspberry Pi and to to send collected measurements to:

* sensor.community
* openSenseMap.org
* InfluxDB
* MQTT broker

## Supported Hardware

* [Nova Fitness SDS011](http://aqicn.org/sensor/sds011/) or compatible connected via `ttyUSB` for dust
* [Bosch BME 2280](https://www.bosch-sensortec.com/bst/products/all_products/bme280) connected via I²C for temperature, humidity and pressure
* [DHT22](https://learn.adafruit.com/dht/overview) for temperature and humidity

## Configuration

Copy the `config.default.toml` to `config.toml` and adjust the settings. It may be neccesary to adjust setting in main.py as well.

## Running a systemd unit

Take a look at the [dusty.service](dusty.service).

## Privileges

On Raspbian the process needs privileges in the groups `i2c` and `dialout`.
