# Wifi-Temperature-Sensor
ESP8266 and AHT21 based Temperature and Humidity Sensor. It's still in very unfinished State, but its working and can also transmit the battery Level to the MQTT Server. 
Battery Lifetime with a 400mah LiFePO4 battery(DURACELL SOLAR BATTERY) should be roughly 3 Weeks with 6 second Inverval. 3 

The PCB Design is available on Circuitmaker:

The Sensor measures the temperature and humidity every 6 seconds (#define interval_us 6000000) and saves it in the RTC Memory. 
Once the RTC Memory is full(about 12 Minutes/126 measurements) it will connect to the configured Wifi and send all values to a MQTT Server in the configured Channel.

The Data is provided as "blob" so it has to be formatted.
The Temperature can be converted with the Formula: Temp = ((Value/65535)*200)-50  or with the Python program "Convert.py"
