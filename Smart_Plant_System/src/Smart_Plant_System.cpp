/* 
 * Project Smart House Plant System
 * Author: Joshua Garcia
 * Date: 
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_GFX.h"
#include "math.h"
#include "Grove_Air_quality_Sensor.h"
#include "Adafruit_BME280.h"
#include "neopixel.h"
#include "credentials.h"
#include <Adafruit_MQTT.h>//subscribe and publish
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);

// Run the application and system concurrently in separate threads
SYSTEM_THREAD(ENABLED);

const int PIN=D16;//Motor pin

unsigned int currentTime;//capacitive sensor
unsigned int lastSec;
unsigned int lastMoiCheck;
unsigned int lastPumpTime;
unsigned int lastMoi;
unsigned int lastAirCheck;
int moi;
 String dateTime , timeOnly; 

 int pin = D13;// dust sensor pin
unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms = 30000;
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;

AirQualitySensor sensor(D14);// Air sensor

Adafruit_BME280 bme;//BME
const int OLED_RESET=-1;
Adafruit_SSD1306 display(OLED_RESET);//OLED
int status;

const int PIXELCOUNT = 11;
Adafruit_NeoPixel pixel( PIXELCOUNT,SPI1, WS2812B);
int perviousPosition;
int startPixel;
int endPixel;
int color;
int maxVal;
int PlantPixel;
void pixelFill( int startPixel,int endPixel,int pixelColor);
float mapFloat(float value, float inMin, float inMax, float outMin, float outMax);

TCPClient TheClient; //publish and subscribe
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 
Adafruit_MQTT_Publish Smarthouseplant = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/smartplantmoisture");
Adafruit_MQTT_Subscribe pump = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/watermotor");
void MQTT_connect();
bool MQTT_ping();
unsigned int last,lastTime;
int subValue;


void setup() {

WiFi.connect();
while (WiFi.connecting()) {
Serial.printf(".");
}
  mqtt.subscribe(&pump);

Serial.begin(9600);
pinMode(PIN,OUTPUT);//Motor pin
pinMode(D11,INPUT);//Capacitive sensor

Particle.syncTime();//Live time
Time.zone(-7);//live time
digitalWrite(PIN,LOW);

if (status == false) {
Serial.printf("BME280 at address 0x%02X failed to start", 0x76); 
}
 waitFor(Serial.isConnected,10000);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
    display.clearDisplay();


pinMode(pin,INPUT);
starttime = millis();

while (!Serial);
Serial.println("Waiting sensor to init...");
delay(10000);
if (sensor.init()) 
{
  Serial.println("Sensor ready.");
} 
else 
{
  Serial.println("Sensor ERROR!");
}
pixel.begin();
pixel.setBrightness(50);//neopixel
pixel.show();
pixel.clear();
}


// loop() runs over and over again, as quickly as it can execute.
void loop() {
MQTT_connect();
MQTT_ping();

currentTime = millis();
if((currentTime-lastMoiCheck)>60000){ //moisture check
  lastMoiCheck = millis();
     if(mqtt.Update()) {
        moi=analogRead(D11);
        Smarthouseplant.publish(moi);
 if (moi >1759){
  digitalWrite(PIN,HIGH);
    //  lastPumpTime = millis();
      delay(500); //temp fix later 
// if((millis()-lastPumpTime)>1000){
        digitalWrite(PIN,LOW);
        Serial.printf("Moisture=%i\n",moi);
 }
 }
}

duration = pulseIn(pin, LOW);// dust sensor
lowpulseoccupancy = lowpulseoccupancy+duration;
if ((millis()-starttime) > sampletime_ms)
{
  ratio = lowpulseoccupancy/(sampletime_ms*10.0);
  concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62;
  Serial.print(lowpulseoccupancy);
  Serial.print(",");
  Serial.print(ratio);
  Serial.print(",");
  Serial.println(concentration);
  lowpulseoccupancy = 0;
  starttime = millis();
}
if((currentTime-lastAirCheck)>60000){ //Air sensor timer
  lastAirCheck = millis();
int quality = sensor.slope();// Air Sensor
Serial.print("Air Quality: ");// Air Sensor
Serial.println(sensor.getValue());

if (quality == AirQualitySensor::FORCE_SIGNAL) {
  Serial.println("High pollution! Force signal active.");
} else if (quality == AirQualitySensor::HIGH_POLLUTION) {
  Serial.println("High pollution!");
} else if (quality == AirQualitySensor::LOW_POLLUTION) {
  Serial.println("Low pollution!");
}
}
display.setCursor(0,0);
display.printf("Moisture=%i\n",moi);//display on oled
display.setCursor(0,10);
display.printf("Air Quality=%i\n",sensor.getValue());//display on oled
display.setCursor(0,20);
display.printf("Dust value=%i,%f,%f\n",lowpulseoccupancy,ratio,concentration);
display.display();
display.clearDisplay();


PlantPixel=-.009*moi+26;
 pixelFill( 0,PlantPixel,0x0000FF);
 Serial.printf("Pixels=%i\n",PlantPixel);


Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(100))) {
    if (subscription == &pump) {
      subValue = atoi((char *)pump.lastread);
      Serial.printf("Received %i from Adafruit.io feed FeedNameB \n",subValue);
    }
  }
  if (subValue ==1)
  {
    digitalWrite(PIN,HIGH);
  }
  else
  {
    digitalWrite(PIN,LOW);
  }

}
//}

 void pixelFill( int startPixel, int endPixel, int pixelColor){
  int i;
for(i=startPixel; i<=endPixel; i++){
    pixel.setPixelColor(i,pixelColor);  
}
pixel.show();
pixel.clear();
}


// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;
 
  // Return if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds...\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds and try again
  }
  Serial.printf("MQTT Connected!\n");
}

bool MQTT_ping() {
  static unsigned int last;
  bool pingStatus;

  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      pingStatus = mqtt.ping();
      if(!pingStatus) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }
  return pingStatus;
}
