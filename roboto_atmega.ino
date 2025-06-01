


#include <Servo.h>
#include <NewPing.h>

// Pin definitions for ultrasonic sensors
#define TRIGGER_PIN_FORWARD 10
#define ECHO_PIN_FORWARD 11
#define TRIGGER_PIN_BACKWARD 26
#define ECHO_PIN_BACKWARD 27
#define TRIGGER_PIN_WATER_TANK 28
#define ECHO_PIN_WATER_TANK 29

#define MAX_DISTANCE 200 // Maximum distance in cm for obstacle detection
const int obstacleThreshold = 15;

// Ultrasonic sensor objects
NewPing sonarForward(TRIGGER_PIN_FORWARD, ECHO_PIN_FORWARD, MAX_DISTANCE);
NewPing sonarBackward(TRIGGER_PIN_BACKWARD, ECHO_PIN_BACKWARD, MAX_DISTANCE);
NewPing sonarWaterTank(TRIGGER_PIN_WATER_TANK, ECHO_PIN_WATER_TANK, MAX_DISTANCE);

#define LIGHT_PIN 13

#define MOTOR_LEFT_FORWARD 2
#define MOTOR_LEFT_BACKWARD 3
#define MOTOR_RIGHT_FORWARD 4
#define MOTOR_RIGHT_BACKWARD 5
#define SPEED_PIN_LEFT 6
#define SPEED_PIN_RIGHT 7

// Flame sensor pin definitions
#define FLAME_SENSOR_left 52
#define FLAME_SENSOR_right 53
#define FLAME_SENSOR_forward_left 50
#define FLAME_SENSOR_forward_right 51

Servo servoMotor;
String inString = "";

// Global variables for distances
unsigned int distanceForward = 0;
unsigned int distanceBackward = 0;
unsigned int distanceWaterTank = 0;
unsigned int flameSensor1 = 0;
unsigned int flameSensor2 = 0;
unsigned int flameSensor3 = 0;
unsigned int flameSensor4 = 0;
unsigned long lastSonarMeasurementTime = 0;  // Last time the sonar sensors were measured
unsigned long sonarMeasurementInterval = 2000; // Interval between sonar measurements (2 seconds)

void setup() {
  Serial3.begin(115200);      // Communication with ESP8266
  Serial.begin(115200);       // Debugging on Serial Monitor

  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
  pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);
  pinMode(SPEED_PIN_LEFT, OUTPUT);
  pinMode(SPEED_PIN_RIGHT, OUTPUT);

  // Set up flame sensor pins as input
  pinMode(FLAME_SENSOR_left, INPUT);
  pinMode(FLAME_SENSOR_right, INPUT);
  pinMode(FLAME_SENSOR_forward_left, INPUT);
  pinMode(FLAME_SENSOR_forward_right, INPUT);

  digitalWrite(LIGHT_PIN, LOW); // Light off by default
  stopMotors();
  servoMotor.attach(12);         // Servo connected to pin 12
  servoMotor.write(90);          // Default servo angle
}

void loop() {
  unsigned long currentMillis = millis();

  // Check if it's time to take another measurement from the sonar sensors
  if (currentMillis - lastSonarMeasurementTime >= sonarMeasurementInterval) {
    lastSonarMeasurementTime = currentMillis;
    object_detectional_distance();  // Measure distances using ultrasonic sensors
    // Check flame sensor values
    readFlameSensors();
  }
  
  serialEvent3(); // Process commands from ESP8266
}

void serialEvent3() {
  while (Serial3.available()) {
    char inChar = Serial3.read();
    inString += inChar;

    if (inChar == ']') {
      processCommand(inString);
      inString = "";  // Clear buffer
    }
  }
}

void object_detectional_distance() {
  // Measure distances from all ultrasonic sensors
  distanceForward = sonarForward.ping_cm();
  distanceBackward = sonarBackward.ping_cm();
  distanceWaterTank = sonarWaterTank.ping_cm();

  // Send ultrasonic data to ESP8266
  sendUltrasonicData();
}

void readFlameSensors() {
  // Read values from flame sensors (HIGH means flame detected, LOW means no flame)
  flameSensor1 = digitalRead(FLAME_SENSOR_left);
  flameSensor2 = digitalRead(FLAME_SENSOR_right);
  flameSensor3 = digitalRead(FLAME_SENSOR_forward_left);
  flameSensor4 = digitalRead(FLAME_SENSOR_forward_right);

  // Send flame sensor data to ESP8266
  sendFlameSensorData();
}

void sendUltrasonicData() {
  // Send ultrasonic data as a JSON string to ESP8266
  String ultrasonicData = "{\"sensor\":\"ultrasonic\"," 
    "\"forward_distance\":" + String(distanceForward) + "," 
    "\"backward_distance\":" + String(distanceBackward) + "," 
    "\"water_tank_distance\":" + String(distanceWaterTank) + "}";

  Serial.println(ultrasonicData);  // Send the data over serial
}

void sendFlameSensorData() {
  // Send flame sensor data as a JSON string to ESP8266
  String flameData = "{\"sensor\":\"flame\"," 
    "\"flame_sensor_left\":" + String(flameSensor1) + ","
    "\"flame_sensor_right\":" + String(flameSensor2) + ","
    "\"flame_sensor_forward_left\":" + String(flameSensor3) + ","
    "\"flame_sensor_forward_right\":" + String(flameSensor4) + "}";

  Serial.println(flameData);  // Send the data over serial
}

void processCommand(String command) {
  if (command.indexOf("[LIGHT_ON]") >= 0) {
    digitalWrite(LIGHT_PIN, HIGH);
  } else if (command.indexOf("[LIGHT_OFF]") >= 0) {
    digitalWrite(LIGHT_PIN, LOW);
  } else if (command.indexOf("[MOVE_FORWARD:") >= 0) {
    int speed = extractSpeed(command, "[MOVE_FORWARD:");
    moveForward(speed);
  } else if (command.indexOf("[MOVE_BACKWARD:") >= 0) {
    int speed = extractSpeed(command, "[MOVE_BACKWARD:");
    moveBackward(speed);
  } else if (command.indexOf("[TURN_LEFT:") >= 0) {
    int speed = extractSpeed(command, "[TURN_LEFT:");
    turnLeft(speed);
  } else if (command.indexOf("[TURN_RIGHT:") >= 0) {
    int speed = extractSpeed(command, "[TURN_RIGHT:");
    turnRight(speed);
  } else if (command.indexOf("[STOP]") >= 0) {
    stopMotors();
  } else if (command.indexOf("[SERVO_ANGLE:") >= 0) {
    int angle = extractSpeed(command, "[SERVO_ANGLE:");
    if (angle >= 0 && angle <= 180) {
      servoMotor.write(angle);
    }
  }
}

int extractSpeed(String command, String keyword) {
  int speedStart = command.indexOf(keyword) + keyword.length();
  int speedEnd = command.indexOf("]", speedStart);
  return command.substring(speedStart, speedEnd).toInt();
}

void moveForward(int speed) {
  analogWrite(SPEED_PIN_LEFT, speed);
  analogWrite(SPEED_PIN_RIGHT, speed);
  digitalWrite(MOTOR_LEFT_FORWARD, HIGH);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
  digitalWrite(MOTOR_RIGHT_FORWARD, HIGH);
  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
}

void moveBackward(int speed) {
  analogWrite(SPEED_PIN_LEFT, speed);
  analogWrite(SPEED_PIN_RIGHT, speed);
  digitalWrite(MOTOR_LEFT_FORWARD, LOW);
  digitalWrite(MOTOR_LEFT_BACKWARD, HIGH);
  digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
  digitalWrite(MOTOR_RIGHT_BACKWARD, HIGH);
}

void turnLeft(int speed) {
  analogWrite(SPEED_PIN_LEFT, speed);
  analogWrite(SPEED_PIN_RIGHT, speed);
  digitalWrite(MOTOR_LEFT_FORWARD, LOW);
  digitalWrite(MOTOR_LEFT_BACKWARD, HIGH);
  digitalWrite(MOTOR_RIGHT_FORWARD, HIGH);
  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
}

void turnRight(int speed) {
  analogWrite(SPEED_PIN_LEFT, speed);
  analogWrite(SPEED_PIN_RIGHT, speed);
  digitalWrite(MOTOR_LEFT_FORWARD, HIGH);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
  digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
  digitalWrite(MOTOR_RIGHT_BACKWARD, HIGH);
}

void stopMotors() {
  analogWrite(SPEED_PIN_LEFT, 0);
  analogWrite(SPEED_PIN_RIGHT, 0);
  digitalWrite(MOTOR_LEFT_FORWARD, LOW);
  digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
}




// #include <Servo.h>
// #include <NewPing.h>

// // Pin definitions for ultrasonic sensors
// #define TRIGGER_PIN_FORWARD 10
// #define ECHO_PIN_FORWARD 11
// #define TRIGGER_PIN_BACKWARD 26
// #define ECHO_PIN_BACKWARD 27
// #define TRIGGER_PIN_WATER_TANK 28
// #define ECHO_PIN_WATER_TANK 29

// #define MAX_DISTANCE 200 // Maximum distance in cm for obstacle detection
// const int obstacleThreshold = 15;

// // Ultrasonic sensor objects
// NewPing sonarForward(TRIGGER_PIN_FORWARD, ECHO_PIN_FORWARD, MAX_DISTANCE);
// NewPing sonarBackward(TRIGGER_PIN_BACKWARD, ECHO_PIN_BACKWARD, MAX_DISTANCE);
// NewPing sonarWaterTank(TRIGGER_PIN_WATER_TANK, ECHO_PIN_WATER_TANK, MAX_DISTANCE);

// #define LIGHT_PIN 13

// #define MOTOR_LEFT_FORWARD 2
// #define MOTOR_LEFT_BACKWARD 3
// #define MOTOR_RIGHT_FORWARD 4
// #define MOTOR_RIGHT_BACKWARD 5
// #define SPEED_PIN_LEFT 6
// #define SPEED_PIN_RIGHT 7

// Servo servoMotor;
// String inString = "";

// // Global variables for distances
// unsigned int distanceForward = 0;
// unsigned int distanceBackward = 0;
// unsigned int distanceWaterTank = 0;
// unsigned long lastSonarMeasurementTime = 0;  // Last time the sonar sensors were measured
// unsigned long sonarMeasurementInterval = 2000; // Interval between sonar measurements (2 seconds)

// void setup() {
//   Serial3.begin(115200);      // Communication with ESP8266
//   Serial.begin(115200);       // Debugging on Serial Monitor

//   pinMode(LIGHT_PIN, OUTPUT);
//   pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
//   pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
//   pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
//   pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);
//   pinMode(SPEED_PIN_LEFT, OUTPUT);
//   pinMode(SPEED_PIN_RIGHT, OUTPUT);

//   digitalWrite(LIGHT_PIN, LOW); // Light off by default
//   stopMotors();
//   servoMotor.attach(12);         // Servo connected to pin 12
//   servoMotor.write(90);          // Default servo angle
// }

// void loop() {
//   unsigned long currentMillis = millis();

//   // Check if it's time to take another measurement from the sonar sensors
//   if (currentMillis - lastSonarMeasurementTime >= sonarMeasurementInterval) {
//     lastSonarMeasurementTime = currentMillis;
//     object_detectional_distance();  // Measure distances using ultrasonic sensors
//   }

//   serialEvent3(); // Process commands from ESP8266
// }

// void serialEvent3() {
//   while (Serial3.available()) {
//     char inChar = Serial3.read();
//     inString += inChar;

//     if (inChar == ']') {
//       processCommand(inString);
//       inString = "";  // Clear buffer
//     }
//   }
// }

// void object_detectional_distance() {
//   // Measure distances from all ultrasonic sensors
//   distanceForward = sonarForward.ping_cm();
//   distanceBackward = sonarBackward.ping_cm();
//   distanceWaterTank = sonarWaterTank.ping_cm();

//   // Send ultrasonic data to ESP8266
//   sendUltrasonicData();
// }

// void sendUltrasonicData() {
//   // Send ultrasonic data as a JSON string to ESP8266
//   String ultrasonicData = "{\"sensor\":\"ultrasonic\","
//     "\"forward_distance\":" + String(distanceForward) + ","
//     "\"backward_distance\":" + String(distanceBackward) + ","
//     "\"water_tank_distance\":" + String(distanceWaterTank) + "}";

//   Serial.println(ultrasonicData);  // Send the data over serial
// }

// void processCommand(String command) {
//   if (command.indexOf("[LIGHT_ON]") >= 0) {
//     digitalWrite(LIGHT_PIN, HIGH);
//   } else if (command.indexOf("[LIGHT_OFF]") >= 0) {
//     digitalWrite(LIGHT_PIN, LOW);
//   } else if (command.indexOf("[MOVE_FORWARD:") >= 0) {
//     int speed = extractSpeed(command, "[MOVE_FORWARD:");
//     moveForward(speed);
//   } else if (command.indexOf("[MOVE_BACKWARD:") >= 0) {
//     int speed = extractSpeed(command, "[MOVE_BACKWARD:");
//     moveBackward(speed);
//   } else if (command.indexOf("[TURN_LEFT:") >= 0) {
//     int speed = extractSpeed(command, "[TURN_LEFT:");
//     turnLeft(speed);
//   } else if (command.indexOf("[TURN_RIGHT:") >= 0) {
//     int speed = extractSpeed(command, "[TURN_RIGHT:");
//     turnRight(speed);
//   } else if (command.indexOf("[STOP]") >= 0) {
//     stopMotors();
//   } else if (command.indexOf("[SERVO_ANGLE:") >= 0) {
//     int angle = extractSpeed(command, "[SERVO_ANGLE:");
//     if (angle >= 0 && angle <= 180) {
//       servoMotor.write(angle);
//     }
//   }
// }

// int extractSpeed(String command, String keyword) {
//   int speedStart = command.indexOf(keyword) + keyword.length();
//   int speedEnd = command.indexOf("]", speedStart);
//   return command.substring(speedStart, speedEnd).toInt();
// }

// void moveForward(int speed) {
//   analogWrite(SPEED_PIN_LEFT, speed);
//   analogWrite(SPEED_PIN_RIGHT, speed);
//   digitalWrite(MOTOR_LEFT_FORWARD, HIGH);
//   digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
//   digitalWrite(MOTOR_RIGHT_FORWARD, HIGH);
//   digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
// }

// void moveBackward(int speed) {
//   analogWrite(SPEED_PIN_LEFT, speed);
//   analogWrite(SPEED_PIN_RIGHT, speed);
//   digitalWrite(MOTOR_LEFT_FORWARD, LOW);
//   digitalWrite(MOTOR_LEFT_BACKWARD, HIGH);
//   digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
//   digitalWrite(MOTOR_RIGHT_BACKWARD, HIGH);
// }

// void turnLeft(int speed) {
//   analogWrite(SPEED_PIN_LEFT, speed);
//   analogWrite(SPEED_PIN_RIGHT, speed);
//   digitalWrite(MOTOR_LEFT_FORWARD, LOW);
//   digitalWrite(MOTOR_LEFT_BACKWARD, HIGH);
//   digitalWrite(MOTOR_RIGHT_FORWARD, HIGH);
//   digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
// }

// void turnRight(int speed) {
//   analogWrite(SPEED_PIN_LEFT, speed);
//   analogWrite(SPEED_PIN_RIGHT, speed);
//   digitalWrite(MOTOR_LEFT_FORWARD, HIGH);
//   digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
//   digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
//   digitalWrite(MOTOR_RIGHT_BACKWARD, HIGH);
// }

// void stopMotors() {
//   analogWrite(SPEED_PIN_LEFT, 0);
//   analogWrite(SPEED_PIN_RIGHT, 0);
//   digitalWrite(MOTOR_LEFT_FORWARD, LOW);
//   digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
//   digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
//   digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
// }










