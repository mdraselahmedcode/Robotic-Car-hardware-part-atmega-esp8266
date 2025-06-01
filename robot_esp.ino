
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// Wi-Fi credentials
const char* ssid = "Sheikh Russell";
const char* password = "12178612*****";

// MQTT Broker details
const char* mqtt_broker = "2c142889200841e4a58e2e4ced8a2569.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_username = "sheikh";
const char* mqtt_password = "Password123";

WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);

// Global variables for sensor data
String forwardDistance = "0";
String backwardDistance = "0";
String waterTankDistance = "0";
String flameSensorLeft = "0";
String flameSensorRight = "0";
String flameSensorForwardLeft = "0";
String flameSensorForwardRight = "0";
String inString = "";

void setup() {
    Serial.begin(115200);

    connectToWiFi();

    wifiClient.setInsecure();  // For testing only
    mqttClient.setServer(mqtt_broker, mqtt_port);
    mqttClient.setCallback(callback);

    connectToMQTT();
}

void loop() {
    if (WiFi.status() != WL_CONNECTED) {
        connectToWiFi();
    }

    if (!mqttClient.connected()) {
        connectToMQTT();
    }

    mqttClient.loop();
    serialEvent();
}

void connectToWiFi() {
    WiFi.begin(ssid, password);
    Serial.println("Connecting to Wi-Fi...");

    unsigned long startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 30000) {
        delay(1000);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWi-Fi connected");
    } else {
        Serial.println("\nWi-Fi connection failed. Restarting...");
        ESP.restart();
    }
}

void connectToMQTT() {
    Serial.println("Connecting to MQTT broker...");
    int retryCount = 0;

    while (!mqttClient.connected()) {
        if (mqttClient.connect("ESP8266Client", mqtt_username, mqtt_password, "robot/status", 0, true, "disconnected")) {
            Serial.println("Connected to MQTT broker");

            mqttClient.publish("robot/status", "connected", true);
            mqttClient.subscribe("test/topic");
        } else {
            retryCount++;
            Serial.print("Failed to connect. State: ");
            Serial.println(mqttClient.state());
            if (retryCount > 5) {
                ESP.restart();
            }
            delay(2000 * retryCount);
        }
    }
}

void serialEvent() {
    while (Serial.available()) {
        char inChar = Serial.read();
        inString += inChar;

        if (inChar == '}') {
            if (inString.length() > 512) {
                inString = "";
                return;
            }
            handleIncomingData(inString);
            inString = "";
        }
    }
}

void handleIncomingData(String sensorData) {
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, sensorData);

    if (error) {
        return;
    }

    // Update ultrasonic sensor values
    if (doc.containsKey("forward_distance")) {
        forwardDistance = doc["forward_distance"].as<String>();
    }
    if (doc.containsKey("backward_distance")) {
        backwardDistance = doc["backward_distance"].as<String>();
    }
    if (doc.containsKey("water_tank_distance")) {
        waterTankDistance = doc["water_tank_distance"].as<String>();
    }

    // Update flame sensor values
    if (doc.containsKey("flame_sensor_left")) {
        flameSensorLeft = doc["flame_sensor_left"].as<String>();
    }
    if (doc.containsKey("flame_sensor_right")) {
        flameSensorRight = doc["flame_sensor_right"].as<String>();
    }
    if (doc.containsKey("flame_sensor_forward_left")) {
        flameSensorForwardLeft = doc["flame_sensor_forward_left"].as<String>();
    }
    if (doc.containsKey("flame_sensor_forward_right")) {
        flameSensorForwardRight = doc["flame_sensor_forward_right"].as<String>();
    }

    // Publish sensor data
    publishCombinedSensorData();
}

void publishCombinedSensorData() {
    StaticJsonDocument<512> jsonDoc;

    // Add ultrasonic sensor data
    jsonDoc["forward_distance"] = forwardDistance;
    jsonDoc["backward_distance"] = backwardDistance;
    jsonDoc["water_tank_distance"] = waterTankDistance;

    // Add flame sensor data
    jsonDoc["flame_sensor_left"] = flameSensorLeft;
    jsonDoc["flame_sensor_right"] = flameSensorRight;
    jsonDoc["flame_sensor_forward_left"] = flameSensorForwardLeft;
    jsonDoc["flame_sensor_forward_right"] = flameSensorForwardRight;

    char jsonString[512];
    serializeJson(jsonDoc, jsonString);

    if (!mqttClient.publish("sensors/all", jsonString)) {
        Serial.println("Failed to publish combined sensor data");
    } else {
        Serial.println("Published combined sensor data: " + String(jsonString));
    }
}

void callback(char* topic, byte* payload, unsigned int length) {
    String message = "";
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    Serial.println("Message received on topic " + String(topic) + ": " + message);
}












// #include <ESP8266WiFi.h>
// #include <WiFiClientSecure.h>
// #include <PubSubClient.h>
// #include <ArduinoJson.h>

// // Wi-Fi credentials
// const char* ssid = "Sheikh Russell";
// const char* password = "12178612*****";

// // MQTT Broker details
// const char* mqtt_broker = "2c142889200841e4a58e2e4ced8a2569.s1.eu.hivemq.cloud";
// const int mqtt_port = 8883;
// const char* mqtt_username = "sheikh";
// const char* mqtt_password = "Password123";

// WiFiClientSecure wifiClient;
// PubSubClient mqttClient(wifiClient);

// String forwardDistance = "0";
// String backwardDistance = "0";
// String waterTankDistance = "0";
// String inString = "";

// void setup() {
//     Serial.begin(115200);

//     connectToWiFi();

//     wifiClient.setInsecure();  // For testing only
//     mqttClient.setServer(mqtt_broker, mqtt_port);
//     mqttClient.setCallback(callback);

//     connectToMQTT();
// }

// void loop() {
//     if (WiFi.status() != WL_CONNECTED) {
//         connectToWiFi();
//     }

//     if (!mqttClient.connected()) {
//         connectToMQTT();
//     }

//     mqttClient.loop();
//     serialEvent();
// }

// void connectToWiFi() {
//     WiFi.begin(ssid, password);
//     Serial.println("Connecting to Wi-Fi...");

//     unsigned long startAttemptTime = millis();
//     while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 30000) {
//         delay(1000);
//         Serial.print(".");
//     }

//     if (WiFi.status() == WL_CONNECTED) {
//         Serial.println("\nWi-Fi connected");
//     } else {
//         Serial.println("\nWi-Fi connection failed. Restarting...");
//         ESP.restart();
//     }
// }

// void connectToMQTT() {
//     Serial.println("Connecting to MQTT broker...");
//     int retryCount = 0;

//     while (!mqttClient.connected()) {
//         if (mqttClient.connect("ESP8266Client", mqtt_username, mqtt_password, "robot/status", 0, true, "disconnected")) {
//             Serial.println("Connected to MQTT broker");

//             mqttClient.publish("robot/status", "connected", true);
//             mqttClient.subscribe("test/topic");
//         } else {
//             retryCount++;
//             Serial.print("Failed to connect. State: ");
//             Serial.println(mqttClient.state());
//             if (retryCount > 5) {
//                 ESP.restart();
//             }
//             delay(2000 * retryCount);
//         }
//     }
// }

// void serialEvent() {
//     while (Serial.available()) {
//         char inChar = Serial.read();
//         inString += inChar;

//         if (inChar == '}') {
//             if (inString.length() > 512) {
//                 inString = "";
//                 return;
//             }
//             handleUltrasonicData(inString);
//             inString = "";
//         }
//     }
// }

// void handleUltrasonicData(String ultrasonicData) {
//     StaticJsonDocument<512> doc;
//     DeserializationError error = deserializeJson(doc, ultrasonicData);

//     if (error) {
//         return;
//     }

//     // Update sensor values
//     if (doc.containsKey("forward_distance")) {
//         forwardDistance = doc["forward_distance"].as<String>();
//     }

//     if (doc.containsKey("backward_distance")) {
//         backwardDistance = doc["backward_distance"].as<String>();
//     }

//     if (doc.containsKey("water_tank_distance")) {
//         waterTankDistance = doc["water_tank_distance"].as<String>();
//     }

//     // Publish combined sensor data
//     publishCombinedSensorData();
// }

// void publishCombinedSensorData() {
//     StaticJsonDocument<256> jsonDoc;
//     jsonDoc["forward_distance"] = forwardDistance;
//     jsonDoc["backward_distance"] = backwardDistance;
//     jsonDoc["water_tank_distance"] = waterTankDistance;

//     char jsonString[256];
//     serializeJson(jsonDoc, jsonString);

//     if (!mqttClient.publish("sensors/ultrasonic", jsonString)) {
//         Serial.println("Failed to publish combined sensor data");
//     } else {
//         Serial.println("Published combined sensor data: " + String(jsonString));
//     }
// }


// void callback(char* topic, byte* payload, unsigned int length) {
//     String message = "";
//     for (int i = 0; i < length; i++) {
//         message += (char)payload[i];
//     }

//     Serial.print("Message arrived on topic: ");
//     Serial.println(topic);
//     Serial.print("Message: ");
//     Serial.println(message);

//     // Handle specific topics
//     if (String(topic) == "robot/control") {
//         if (message == "start") {
//             Serial.println("Starting robot...");
//         } else if (message == "stop") {
//             Serial.println("Stopping robot...");
//         }
//     } else if (String(topic) == "test/topic") {
//         Serial.println("Message received on test/topic: " + message);
//     }
// }
















