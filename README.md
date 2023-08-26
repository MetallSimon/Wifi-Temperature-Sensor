# Wifi-Temperature-Sensor
ESP8266 and AHT21 based Temperature and Humidity Sensor

The PCB Design is available on Circuitmaker:

The Sensor measures the temperature and humidity every 6 seconds (#define interval_us 6000000) and saves it in the RTC Memory. 
Once the RTC Memory is full(about 12 Minutes/126 measurements) it will connect to the configured Wifi and send all values to a MQTT Server in the configured Channel.

The Data is provided as "blob" so it has to be formatted.
