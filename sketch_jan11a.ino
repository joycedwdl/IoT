#include <WiFi.h>
#include <ArduinoMqttClient.h>
#include "secrets.h"

#define GREEN_LED_PIN 25
#define BLUE_LED_PIN 32
#define RED_LED_PIN 33
#define BUTTON_PIN 26
#define MOTION_SENSOR_PIN 27
#define TRANSISTOR_PIN 13



WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);
const int port = 1883; // MQTT port
const char topic[] = "joycedewildedeligny/securityState"; // MQTT topic used for saving the securityState
bool securityState = false; // Whether the security system is turned on or off
bool transistorState = false; // Whether the transistor is turned on or off
bool stateReceived = false; // If the securityState was already received from mqtt
int previousState = HIGH; // Previous button  state
unsigned long lastDebounceTime = 0; // Used to debounce the button
unsigned long debounceTime = 100; // Debounce time for the button


void changeSecurityState(bool newState) { // Turn security system on or off
  securityState = newState; // Change state variable

  digitalWrite(RED_LED_PIN, securityState); // Turn light on or off
  digitalWrite(BLUE_LED_PIN, securityState);
  digitalWrite(GREEN_LED_PIN, securityState);

  mqttClient.beginMessage(topic,true,0); // Publish changes to mqtt to be able to resume after a restart
  mqttClient.print(securityState);
  mqttClient.endMessage();
}


void onMqttMessage(int messageSize) { // Incoming message function
  if (stateReceived) // If the broker already sent the state, don't run again
    return;
  stateReceived = true;
  
  String message = ""; // Get the message from mqtt
  while (mqttClient.available()) {
    message.concat((char) mqttClient.read());
  }
  changeSecurityState(message); // Change the security state to the received mqtt value
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(GREEN_LED_PIN, OUTPUT); // Set all pin modes
  pinMode(BLUE_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(MOTION_SENSOR_PIN, INPUT);
  pinMode(TRANSISTOR_PIN, OUTPUT);

  WiFi.mode(WIFI_STA); // Set wifi client mode so you can connect to a wifi network
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD); // Connect to the wifi network 

  while (WiFi.status() != WL_CONNECTED) { // Try connecting again if the connection failed
    Serial.println("Can't connect to wifi");
    delay(1000);
  }

  Serial.println("Connected to wifi");

  bool MQTTconnected = false;
  while (!MQTTconnected) { // Try connecting to the mqtt broker until the connection succeeds
    if (!mqttClient.connect(BROKER_IP, port))
      delay(1000);
    else {
      MQTTconnected = true;
      Serial.println("Connected to MQTT");
    }
  }

  mqttClient.onMessage(onMqttMessage); // Subscribe to the topic
  mqttClient.subscribe(topic);
}

void loop() {
  // put your main code here, to run repeatedly:
  mqttClient.poll(); // Send and receive mqtt messages

  int buttonState = digitalRead(BUTTON_PIN); // Read button state

  if ((millis() - lastDebounceTime) > debounceTime) { // Debounce the button
    if(previousState == HIGH && buttonState == LOW) { // If button is pressed
      Serial.println("The button is pressed");
      lastDebounceTime = millis(); // Update debounce time

      if (transistorState) { // If the transistor is turned on, turn it off
        transistorState = false;
        digitalWrite(TRANSISTOR_PIN, false);
      }
      
      changeSecurityState(!securityState); // Toggle the security state
    }
  }

  if (securityState && !transistorState && digitalRead(MOTION_SENSOR_PIN)) { // If security system is on, transistor is off and motion is detected
    digitalWrite(TRANSISTOR_PIN, true); // Turn on transistor
    transistorState = true;
  }

  previousState = buttonState; // Save previous button state
}
