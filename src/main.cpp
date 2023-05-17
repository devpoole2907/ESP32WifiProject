#include <Arduino.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <AsyncMqttClient.h>
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}

#define WIFI_SSID "a wifi ssid"
#define WIFI_PASS "awifipassword"

const char* mqttServer = "mymqttipaddress";
const uint16_t mqttPort = 1883;
const char* mqttUser = "homeassistant";
const char* mqttPassword = "yeahnotputtingthishere";

const char* temperatureTopic = "esp32/sensor/temperature";
const char* humidityTopic = "esp32/sensor/humidity";
const char* motionTopic = "esp32/sensor/motion";

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

unsigned long previousMillis = 0;   // stores last time temperature was published
const long interval = 10000;

DHT dht(4, DHT11);

float temp;
float humidity;

Adafruit_SSD1306 display(128, 64, &Wire);

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  
  mqttClient.connect();
}

void connectToWifi(){
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void updateDisplay(float temp, float humidity) {
  display.clearDisplay();
  
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("Temperature: ");
  display.setTextSize(2);
  display.setCursor(0,10);
  display.print(temp);
  display.print(" ");
  display.setTextSize(1);
  display.cp437(true);
  display.write(167);
  display.setTextSize(2);
  display.print("C");

  display.setTextSize(1);
  display.setCursor(0, 35);
  display.print("Humidity: ");
  display.setTextSize(2);
  display.setCursor(0, 45);
  display.print(humidity);
  display.print(" %");

  display.display();
}


void setup() {
//initialises the serial communication
  Serial.begin(115200);
dht.begin();

mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));


  WiFi.onEvent(WiFiEvent);

mqttClient.onDisconnect(onMqttDisconnect);

mqttClient.setServer(mqttServer, mqttPort);

mqttClient.setCredentials(mqttUser, mqttPassword);

connectToWifi();

  delay(2000);

  pinMode(5, INPUT);
  delay(60 * 1000);
  
  pinMode(LED_BUILTIN, OUTPUT);


  
  Serial.println("Booting up...");

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }

  display.clearDisplay();
  display.setTextColor(WHITE);



}

bool isConnected = false;
unsigned long lastWiFiCheck = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastMotionUpdate = 0;
const unsigned long displayUpdateInterval = 5000;
const unsigned long motionUpdateInterval = 200;

void loop() {

  if(millis() - lastWiFiCheck >= 1000){
    lastWiFiCheck = millis();
    if (WiFi.status() == WL_CONNECTED && !isConnected)
    {
      Serial.println("Connected");
      digitalWrite(LED_BUILTIN, HIGH);
      isConnected = true;
  }

  if (WiFi.status() != WL_CONNECTED){
    Serial.println(".");
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    isConnected = false;
  }
  }

  if(millis() - lastDisplayUpdate >= displayUpdateInterval){
  lastDisplayUpdate = millis();
    
  humidity = dht.readHumidity();
  temp = dht.readTemperature();

  if (!isnan(humidity) && !isnan(temp)) {

    Serial.print("Humidity: ");
  Serial.println(humidity);
  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.println(" *C");

  updateDisplay(temp, humidity);

    uint16_t packetIdPub1 = mqttClient.publish(temperatureTopic, 1, true, String(temp).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", temperatureTopic, packetIdPub1);
    Serial.printf("Message: %.2f \n", temp);

    // Publish an MQTT message on topic esp32/dht/humidity
    uint16_t packetIdPub2 = mqttClient.publish(humidityTopic, 1, true, String(humidity).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", humidityTopic, packetIdPub2);
    Serial.printf("Message: %.2f \n", humidity);

  } else {
    Serial.println("Failed to recieve any temperature measurements");
    return;
  }



  
}

  if (millis() - lastMotionUpdate >= motionUpdateInterval) {
  lastMotionUpdate = millis();
  int motionVal = digitalRead(5);
  if (motionVal == HIGH) {
    Serial.println("Motion detected");
    uint16_t packetIdPubMotion = mqttClient.publish(motionTopic, 1, true, "1"); // Publish "1" when motion is detected
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i\n", motionTopic, packetIdPubMotion);

    display.ssd1306_command(SSD1306_DISPLAYON);
    updateDisplay(temp, humidity);
  } else {
    Serial.println("No motion detected");
    uint16_t packetIdPubMotion = mqttClient.publish(motionTopic, 1, true, "0"); // Publish "0" when no motion is detected
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i\n", motionTopic, packetIdPubMotion);

    display.ssd1306_command(SSD1306_DISPLAYOFF);
  }
}



  

}


