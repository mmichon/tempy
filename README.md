# Tempy - Remote Temperature Sensor Over LoRa

This project implements a remote temperature sensor with temperature and humidity data delivered over LoRa. Hardware (sensor/sender and receiver/display) costs around $30 USD. Data is sampled and sent every second from the sender device, and decoded and displayed on the receiver's display.

## Hardware Needed
<img src="https://github.com/mmichon/tempy/blob/master/receiver.jpeg"> <img src="https://github.com/mmichon/tempy/blob/master/sender.jpeg">

* 2 ESP32 boards with SX1276 LoRa radio, like [this one](https://www.aliexpress.com/item/2pcs-of-TTGO-LORA32-868-915Mhz-SX1276-ESP32-Oled-display-Bluetooth-WIFI-Lora-development-board/32841743946.html?spm=a2g0s.9042311.0.0.eiWldj)
* SHT3XD sensor wired to the I2C bus, like [this one](https://smile.amazon.com/HiLetgo-Temperature-Humidity-Interface-GY-SHT31-D/dp/B07ZSZW92J/ref=sr_1_1?crid=275Q9TP5PVBFQ&keywords=sht3x-d&qid=1661912039&sprefix=sht3x-d%2Caps%2C138&sr=8-1). For the TTGO Lora v1 bored linked above, hook up SDA of the SHT3XD to pin 21, and SCL to pin 22.
* Optionally two 3D printed cases for the units, like [these](https://www.thingiverse.com/search?q=ttgo+lora&type=things&sort=relevant&page=1)

## Configuration
1. Install PlatformIO.
1. `pio run -t upload` in the `send` and `recv` directories for each of the units, respectively.
