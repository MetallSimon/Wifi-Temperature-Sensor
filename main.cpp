#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <PubSubClient.h>
#include "user_interface.h" // for keeping time across light sleep based on the RTC

/*
-Use Wifi Manager or https://hieromon.github.io/AutoConnect/credit.html
to connect to Wifi and configure all the settings 
- does it need  WiFi.persistent(false);?
-probably also use client.subscribe("inTopic"); te set additional stuff
*/


/*
Information link List:
AHT21 :
- http://www.aosong.com/userfiles/files/media/Data%20Sheet%20AHT21.pdf
- https://github.com/enjoyneering/AHTxx


The AHT21 Sensor (with LDO :C) i used:
- https://www.electroschematics.com/temperature-sensor/

ESP RTC Memory (system_rtc_mem_write):
- https://www.espressif.com/sites/default/files/documentation/2c-esp8266_non_os_sdk_api_reference_en.pdf

ESP Light and Deep Sleep:
- https://www.mischianti.org/2019/11/21/wemos-d1-mini-esp8266-the-three-type-of-sleep-mode-to-manage-energy-savings-part-4/
- https://kevinstadler.github.io/notes/esp8266-deep-sleep-light-sleep-arduino/
- https://github.com/SensorsIot/ESP8266-Longterm-Sensor-Hourly
- https://github.com/esp8266/Arduino/blob/master/libraries/esp8266/examples/LowPowerDemo/LowPowerDemo.ino

MQTT:
- https://makesmart.net/esp8266-d1-mini-mqtt/
- https://pubsubclient.knolleary.net/api
- https://www.youtube.com/watch?v=hyJhKWhxAxA&t=0s
*/


/*
system_rtc_mem_write   in  ESP8266 Non-OS SDK API Reference
Data read/write accesses to the RTC memory must be word aligned (4 bytes boundary
aligned). Parameter src_addr means block number(4 bytes per block). For example, to
read data from the beginning of user data area, src_addr will be 256/4=64, save_size will
be data length
Read user data from RTC memory. Only user data area should be accessed by the user.
|<------system data (256 bytes)------->|<-----------------user data (512 bytes)--------------->
*/

/*
Function description:
The function "ReadandconvertSensor" will read the Data from the Sensor, trim it to fit into one RTC Memory block(32 bit) and store it there
Once the RTC Memory is full, the  function "mqtt_transmit" will be called.
  This Function will read all the Data stored in RTC Memory and transmit it via MQTT.
  This Function also calls the function "connect_MQTT" in the beginning.
The current memory position(circular pointer?) is stored in RTC Memory position 65 and is called "RTC_Adress"
*/



/*------------------------Settings section start------------------------*/
//WIFI Login Data
const char wifissid[] ="Wifi-SSID";                           //your wifi ssid
const char wifipassw[] ="Wifi-Password";                    //your wifi password
const int32_t wifichannel = 0 ;                                   //wifi channel. 0 for auto detect
const uint8_t wifibssid[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; //wifi bssid(mac adress)   

//Settings for Static IP
//#define usestaticIP       //uncomment to use static IP settings
IPAddress staticIP(192,168,0,202);
IPAddress gateway(192,168,0,1);
IPAddress subnet(255,255,255,0);

//MQTT Broker 
const char* mqtt_server = "192.168.178.201";
const String clientId = "ESP8266Client-12345";  
const uint8_t devicelowerid =1;   //Lower identifier used for device id
const uint8_t deviceupperid =0;   //Upper identifier used for device type
#define mqtt_topic_Temp "Temperatur" 
#define mqtt_topic_Hum  "Humidity" 
#define mqtt_topic_Batt "Voltage" 
//add if not defined dont use
#define mqtt_topic_Time "Time" 

//Sensor Settings
#define interval_us   6000000   //wake and measure interval in us 
#define Sensor_Power  14
#define Sensor_SCL    5
#define Sensor_SDA    4
#define Sensoradress 0x38 //AHT21 i2C Adress

/*------------------------Settings section end------------------------*/

WiFiClient espClient;
PubSubClient client(espClient);
ADC_MODE(ADC_VCC);

/*This function might be a little Confusing.Temperature data is 20bit andd humidity also.
RTC memory is word aligned, so 32bit. we could just save temp in one memory block and humidity 
in the next one, but that would waste 12bit(32-20) per memory block
We can use 3 Memory Blocks to save 2x temperature and 2x humidity
but what i do instead, lower the resolution to 16bit, which is still plenty
- Optimize this by lowering temperature range for higher precision
- add light sleep to this function
*/
/*
void wakeupCallback() {
  Serial.println("Das hat wohl nicht funktioniert");
  // this flush is crucial, possibly because it is a blocking command that
  // allows the CPU to break out of the delay()? see output below
  Serial.flush();
}
*/

 //no idea, just copied from aosong example
void initSensorRegister() {
  uint8_t Registers[] ={0x1B, 0x1C, 0x1E};
  uint8_t ReadData[2];
  for (uint8_t i = 0; i < 3; i++){
    Wire.beginTransmission(Sensoradress);
    Wire.write(0x70);
    Wire.write(Registers[i]);
    Wire.write(0x00);
    Wire.write(0x00);
    if (Wire.endTransmission(true) != 0) {/*error here*/}
    delay(5);
    Wire.beginTransmission(Sensoradress);
    Wire.write(0x71); 
    if (Wire.endTransmission(true) != 0) {/*error here*/}
    Wire.requestFrom(Sensoradress, (int)3, (int)true);
    ReadData[0] = Wire.read();//first byte not used??
    ReadData[0] = Wire.read();
    ReadData[1] = Wire.read();
    delay(10);
    Wire.beginTransmission(Sensoradress);
    Wire.write(0x70);
    Wire.write(0xB0|Registers[i]);
    Wire.write(ReadData[0]);
    Wire.write(ReadData[1]);
    if (Wire.endTransmission(true) != 0) {/*error here*/}
  }
  Serial.println();
  Serial.println("Calibration done");
}

uint32_t ReadandconvertSensor(){
  delay(8);  //wait for reset?
  digitalWrite (Sensor_Power, HIGH); // Sensor Power High.
  delay(7);  //When the sensor is just powered on, MCU gives priority to VDD power supply, and SCL and SDA high levels can be set after 5ms.
  uint8_t  SensorData[7];
  //digitalWrite(Sensor_SCL, HIGH);       //"(SCL is high at this time)" what is this documentation??
  delay(110);                             //After power-on, the sensor needs ≥100ms time (SCL is high at this time) to reach the idle state + some margin for capacitor
  Wire.begin(SDA, SCL);
  Wire.setClock(200000);  //I2C bus speed 200kHz
  Wire.setClockStretchLimit(1000); 
  Wire.beginTransmission(Sensoradress);
  Wire.write(0x71);                     //Before reading the temperature and humidity value, get a byte of status word by sending 0x71.
  if (Wire.endTransmission(true) != 0) {/*error here*/}
  Wire.requestFrom(Sensoradress, (int)1, (int)true); 
  SensorData[0] = Wire.read();
  uint8_t count = 0;
  if((SensorData[0]&0x18)!=0x18){
    initSensorRegister();
  }
  Wire.beginTransmission(Sensoradress);
  Wire.write(0x71);                     //Before reading the temperature and humidity value, get a byte of status word by sending 0x71.
  if (Wire.endTransmission(true) != 0) {/*error here*/}
  Wire.requestFrom(Sensoradress, (int)1, (int)true); 
  SensorData[0] = Wire.read();
  if((SensorData[0]&0x18)!=0x18){
    Serial.println("trotzdem kaputt");
    count = 60;
  }
  Wire.beginTransmission(Sensoradress);
  delay(10);
  Wire.write(0xAC);       // Wait 10ms to send the 0xAC command (trigger measurement). This command parameter has two bytes,
  Wire.write(0x33);       // the first byte is 0x33
  Wire.write(0x00);       //and the second byte is 0x00
  if (Wire.endTransmission(true) != 0) {/*error here*/}
  
  delay(80);               //Wait 80ms for the measurement to be completed
  
  Wire.requestFrom(Sensoradress, (int)1, (int)true); //if the read status word Bit[7] is 0, it means the measurement is completed, 
  SensorData[0] = Wire.read();
  while((( SensorData[0]&0x80)==0x80)){
    if (count>50){
      break;
    }
    Wire.beginTransmission(Sensoradress);
    Wire.write(0x71);
    if (Wire.endTransmission(true) != 0) {/*error here*/}
    Wire.requestFrom(Sensoradress, (int)1, (int)true); 
    SensorData[0] = Wire.read();
    count++;
    delay(2);
    yield();
  } //some othere Error
  Wire.requestFrom(Sensoradress, (int)6, (int)true);  //After receiving six bytes, the next byte is CRC check data, which the user can read as needed
  for (uint8_t i = 0; i < 6; i++)
  {
    SensorData[i] = Wire.read();
  }


uint32_t temperature   = SensorData[3] & 0x0F;             
           temperature <<= 8;
           temperature  |= SensorData[4];
           temperature <<= 8;
           temperature  |= SensorData[5];
Serial.print("temperatur ist:");
Serial.println(((float)temperature / 0x100000) * 200 - 50);

uint32_t humidity   = SensorData[1];                         
           humidity <<= 8;
           humidity  |= SensorData[2];
           humidity <<= 4;
           humidity  |= SensorData[3] >> 4;

  humidity  |= 0x08;          //rounding
  temperature  |= 0x08;       //rounding
  temperature >>= 4;          //remove 4 Bits
  humidity >>= 4;             //remove 4 Bits
  humidity <<= 16;
  temperature  |= humidity;           //combine both
  if(count == 60){  //measurement incomplete. somethin is wrong, so we return 100 as error
    temperature= 100;
  }else if(count > 50){ //reading took to long. error 101
    temperature= 101;
  }
  return temperature;
}



void connect_MQTT() {
  // Loop until we're reconnected -> but only a few times so the batttery still lasts a while, so there is timeout
  uint8_t timeout = 0;
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      if(timeout >= 5){
        ESP.deepSleep(interval_us, WAKE_RF_DEFAULT); 
        // try again later
      }
      // Wait 0.5 seconds before retrying
      delay(500);
      timeout++;
    }
  }
}


/*Transmit the Data stored in RTC Memory
- add status information (battery level with wifi modem active, unsucessfull transmit / Wifi)
*/
bool mqtt_transmit(){
  client.setServer(mqtt_server, 1883);
  if (!client.connected()) {
    connect_MQTT();
  }
   client.loop();
   //snprintf (msg, MSG_BUFFER_SIZE, "hello world #%ld", millis());
   client.beginPublish(mqtt_topic_Temp, 252, false);
   uint32_t transmitbuffer = 0;
   uint8_t transmit1bufferchar[2]; // transmit our Data as char, don't know a simpler way at the moment
   uint8_t transmit2bufferchar[250]; 
   client.write(deviceupperid);
   client.write(devicelowerid);
   for(uint8_t i = 0; i < 125; i++){
    system_rtc_mem_read((i+66),&transmitbuffer, 4);
    //check and improve this part
    transmit1bufferchar[0] = transmitbuffer;
    transmit1bufferchar[1] = transmitbuffer>>8;
    transmit2bufferchar[(i*2)] = transmitbuffer>>16;
    transmit2bufferchar[(i*2)+1] = transmitbuffer>>24;
    client.write(transmit1bufferchar[1]);
    client.write(transmit1bufferchar[0]);
   }
  client.endPublish();


  client.beginPublish(mqtt_topic_Hum, 252, false);
  client.write(deviceupperid);
  client.write(devicelowerid);
  //this can start from 0 instead of 66
  for(uint8_t i = 0; i < 125; i++){
    client.write(transmit2bufferchar[(i*2)+1]);
    client.write(transmit2bufferchar[(i*2)]);
   }
  client.endPublish();

  /*vvvvvvvvvvv Transmit Block for Battery Status vvvvvvvvvvv*/
  system_rtc_mem_read((65),&transmitbuffer, 4);
  transmit1bufferchar[0] = transmitbuffer>>16;
  transmit1bufferchar[1] = transmitbuffer>>24;
  client.beginPublish(mqtt_topic_Batt, 4, false);
  client.write(deviceupperid);
  client.write(devicelowerid);
  client.write(transmit1bufferchar[1]);
  client.write(transmit1bufferchar[0]);
  client.endPublish();

  /*vvvvvvvvvvv Transmit Block for connection Time vvvvvvvvvvv*/
  transmitbuffer = millis();
  transmit1bufferchar[0] = transmitbuffer;
  transmit1bufferchar[1] = transmitbuffer>>8;
  client.beginPublish(mqtt_topic_Time, 4, false);
  client.write(deviceupperid);
  client.write(devicelowerid);
  client.write(transmit1bufferchar[1]);
  client.write(transmit1bufferchar[0]);
  client.endPublish();
  Serial.println("mqtt finished");
  return true;
}



void setup() {
    pinMode(Sensor_Power, OUTPUT); //Sensor Power
    pinMode(Sensor_SCL, OUTPUT); //Sensor SCL
    pinMode(Sensor_SDA, OUTPUT); //Sensor SDA
    digitalWrite(Sensor_Power, LOW);  //Sensor Power Low for startup
    digitalWrite(Sensor_SCL, LOW);  //Sensor SCL Low for startup
    digitalWrite(Sensor_SDA, LOW);  //Sensor SDA Low for startup
    client.setServer(mqtt_server, 1883);
    Serial.begin(9600); // open the serial port at 9600 bps:
}


void loop() {

  Serial.print(client.getBufferSize());
  uint32_t  RTC_Adress = 0; 
  system_rtc_mem_read(65,&RTC_Adress, 4);
  //extract the rtc adress, remove voltage
  RTC_Adress = RTC_Adress & 0x000000FF; 
  //Testing some stuff
  if(RTC_Adress >= 180){RTC_Adress = 65;}
  if(RTC_Adress >= 191){
    RTC_Adress = 65;  //not the best way to implement, change this
    WiFi.mode(WIFI_STA);
    #ifdef usestaticIP
      WiFi.config(staticIP, gateway, subnet);
    #endif
    WiFi.begin(wifissid, wifipassw, wifichannel, wifibssid);
    Serial.println(WiFi.getMode());
    uint8_t timeout = 0;
    while (WiFi.status() != WL_CONNECTED)
      {
        delay(500);
        Serial.print(".");
        timeout++ ;
        // connecting takes to long, reset and omit Data
        if(timeout >10){
          system_rtc_mem_write (65, &RTC_Adress, 4);
          ESP.deepSleep(interval_us, WAKE_RF_DISABLED);
          yield(); 
        }
      }
    Serial.println();

    Serial.print("Connected, IP address: ");
    Serial.println(WiFi.localIP());
    if(mqtt_transmit()){  //see XXXXX comment below.
      system_rtc_mem_write (65, &RTC_Adress, 4);
      Serial.println("disabling wifi now");
      /*Dont use Sensor and Wifi at the same time -> less peak current -> preserve battery*/
      /*XXXXXXXXXXX if using deepSleep here without delay before, mqtt_transmit isnt working??????? XXXXXXXXXXXX*/
      delay(15);
      ESP.deepSleep((interval_us/2), WAKE_RF_DISABLED);
      yield(); 
    }
  }
  else if(RTC_Adress < 66){
      //in Case we come from an unknown state
       RTC_Adress = 66;
  }
  else{
    RTC_Adress++ ; 
  }
  Serial.print("RTC-Adresse ist:");
  Serial.println(RTC_Adress);
  //Serial.println(RTC_Adress);
  uint32_t convertedData = ReadandconvertSensor();
  Serial.println(convertedData);
  system_rtc_mem_write (65, &RTC_Adress, 4);
  system_rtc_mem_write (RTC_Adress, &convertedData, 4);
  digitalWrite(Sensor_SCL, LOW);  //Sensor SCL Low 
  digitalWrite(Sensor_SDA, LOW);  //Sensor SDA Low 
  digitalWrite (Sensor_Power, LOW); // Sensor Power 
  if(RTC_Adress >= 191){
    uint16_t measuredvolts = ESP.getVcc();
    RTC_Adress = (measuredvolts<<16)| RTC_Adress;
    system_rtc_mem_write (65, &RTC_Adress, 4);
    Serial.println("next wifi wake");
    Serial.print("Voltage is:");
    Serial.println(measuredvolts);
    ESP.deepSleep((interval_us/2), WAKE_RF_DEFAULT);  // interval_us/2 because we sleep after wifi modem was used also see above
  }else{
    ESP.deepSleep(interval_us, WAKE_RF_DISABLED);    // mode is one of WAKE_RF_DEFAULT, WAKE_RFCAL, WAKE_NO_RFCAL, WAKE_RF_DISABLED
  }
  yield();                                            // pass control back to background processes to avoid looping
}