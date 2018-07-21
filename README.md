# RoomMonitor

Software/firmware for DYI room conditions monitor.

- ESP8266 (on a developer board), wifi connection & brain
- SHT30 - temperature and humidity sensor
- BMP280 - atmospheric pressure sensor
- BH1750 - light intensity sensor.

All sesnors communicate via I2C bus. Data are sent to io.adafruit.com and MQTT is used for data transfer.

