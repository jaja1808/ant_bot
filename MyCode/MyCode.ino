#include <Servo.h>
#include <SoftwareSerial.h>

//Definition of Variables
Servo leftMotor;
Servo rightMotor;

SoftwareSerial nodeMCU(10, 11);  // Rx, Tx pins

//Time variables
const int serPeriod = 250;
unsigned long serialDelay = 0;
unsigned long turnPeriod;
unsigned long turnStart;

// Delays
const int loopPeriod = 250;
unsigned long LoopDelay = 0;
unsigned long startTime = 0;
unsigned long duration = 30000;                //remember to change  below
unsigned long newDuration = duration + 12003;  //modify the value

//Sonor Pins
const int _trigger = 8;
const int _echo = 9;

//Start Button
const int btnPin = 2;
int btnState = 0;

//Indication Leds
const int left_led = 4;
const int right_led = 5;
const int reverse_led = 6;
const int read_dist_led = 13;
const int forward_led = 7;
const int backward_led = 3;


//Sensor variables
int ult_distance;
int ult_duration;

// Coordinates for navigation
float currentX = 0.0;
float currentY = 0.0;
float targetX = 0.0;
float targetY = 0.0;

// Constants
#define FORWARD 0
#define LEFT 1
#define RIGHT 2
#define EXPLORE 3
#define FOOD 4
#define HOME 5

// State and Mode of the robot
int state = FORWARD;
int mode = EXPLORE;

// Array for movement
int* myPath = NULL;
int size = 0;

// MAKING THE FUNCTION OF A VECTOR LIKE ARRAY
void addElement(int element) {
  // Allocate memory for a new array that's one element larger than the current array
  int* temp = new int[size + 1];
  // Copy the current array into the new array
  for (int i = 0; i < size; i++) {
    temp[i] = myPath[i];
  }
  // Add the new element to the end of the new array
  temp[size] = element;
  // Free memory used by the old array
  delete[] myPath;
  // Point the myArray pointer to the new array
  myPath = temp;
  // Update the size of the array
  size++;
}

// Function to reset myPath and clear its values
void resetPath() {
  // Free memory used by the current array
  delete[] myPath;
  // Set myPath pointer to NULL
  myPath = NULL;
  // Reset the size to 0
  size = 0;
}

// PRINTING THE ARRAY CONTENT
void printArray() {
  Serial.println("Path:");
  for (int i = 0; i < size; i++) {
    Serial.print(myPath[i]);
    Serial.print(" ");
  }
  Serial.println();
}

// READ THE COORDINATES OFF OF WIFI MODULE
void readCoordinates() {
  if (nodeMCU.available()) {
    // Read the data sent from the NodeMCU
    String data = nodeMCU.readStringUntil('\n');

    // Parse the data and extract the coordinates
    sscanf(data.c_str(), "%f,%f,%f,%f", &targetX, &targetY, &currentX, &currentY);

    // Print the coordinates for verification
    Serial.print("Received coordinates: targetX=");
    Serial.print(targetX);
    Serial.print(", targetY=");
    Serial.print(targetY);
    Serial.print(", currentX=");
    Serial.print(currentX);
    Serial.print(", currentY=");
    Serial.println(currentY);
    mode = FOOD;

  } else {
    mode = EXPLORE;
  }
}

// Setup of pins and baud
void setup() {

  Serial.begin(9600);
  nodeMCU.begin(9600);

  // Setting the motors
  pinMode(_trigger, OUTPUT);
  pinMode(_echo, INPUT);
  pinMode(btnPin, INPUT);
  leftMotor.attach(12);
  rightMotor.attach(13);

  //LED for showing the functions
  pinMode(left_led, OUTPUT);
  pinMode(right_led, OUTPUT);
  pinMode(reverse_led, OUTPUT);
  pinMode(read_dist_led, OUTPUT);
  pinMode(backward_led, OUTPUT);
  pinMode(forward_led, OUTPUT);
}

// SENSOR READING FUNCTION
void readSensor() {
  // indicator on
  digitalWrite(read_dist_led, HIGH);

  //send impulse every 10 Microseconds
  digitalWrite(_trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(_trigger, LOW);
  //read the echo
  ult_duration = pulseIn(_echo, HIGH);
  ult_distance = (ult_duration / 2) / 29;

  // indicator off
  digitalWrite(read_dist_led, LOW);
}

// SIMPLE MOVEMENT FUNCTIONS

void turnRight() {
  // indicator on
  digitalWrite(right_led, HIGH);

  //turning period conditions
  turnPeriod = 200;
  turnStart = millis();
  //turning
  while ((millis() - turnStart) < turnPeriod) {
    rightMotor.write(40);
    leftMotor.write(40);
  }
  // indicator off
  digitalWrite(right_led, LOW);
}

void moveBackward() {
  // indicator on
  digitalWrite(backward_led, HIGH);

  //turning period conditions
  turnPeriod = 10;
  turnStart = millis();
  //backward
  while ((millis() - turnStart) < turnPeriod) {
    rightMotor.write(82);
    leftMotor.write(100.005);
  }
  // indicator off
  digitalWrite(backward_led, LOW);
}

void moveForward() {
  // indicator on
  digitalWrite(forward_led, HIGH);

  turnPeriod = 50;
  turnStart = millis();
  //turning
  while ((millis() - turnStart) < turnPeriod) {
    rightMotor.write(100.3); // changing the speed to add 0.3
    leftMotor.write(79.5);
  }
  // indicator off
  digitalWrite(forward_led, LOW);
}

void turnLeft() {
  // indicator on
  digitalWrite(left_led, HIGH);

  //turning period conditions
  turnPeriod = 200;
  turnStart = millis();
  //turning
  while ((millis() - turnStart) < turnPeriod) {
    rightMotor.write(140);
    leftMotor.write(140);
  }
  // indicator off
  digitalWrite(left_led, LOW);
}

void rotation() {

  turnPeriod = 1000;
  turnStart = millis();
  //turning

  while ((millis() - turnStart) < turnPeriod) {
    rightMotor.write(120);
    leftMotor.write(120);
  }
}

void stopped() {
  rightMotor.write(90);
  leftMotor.write(90);
}

// COMPLEX MOVEMENT FUNCTIONS

void movement() {
  // move forward until there is an obtacle
  if (state == FORWARD) {
    addElement(state);
    Serial.print(state);

    if (ult_distance > 20 || ult_distance < -5) {
      moveForward();
    } else {

      // creating a random movement
      int random = rand() % 10;
      if (random > 5) {
        state = LEFT;
        addElement(state);
        Serial.print(state);

      } else {
        state = RIGHT;
        addElement(state);
        Serial.print(state);
      }
    }
  } else if (state == LEFT) {
    turnLeft();
    state = FORWARD;
    addElement(state);
    Serial.print(state);

  } else if (state == RIGHT) {
    turnRight();
    state = FORWARD;
    addElement(state);
    Serial.print(state);
  }
}

void reverseDir() {
  // after turning 360 degrees
  pinMode(reverse_led, HIGH);  // indicator on

  for (int i = size - 1; i >= 0; i--) {
    readSensor();
    state = myPath[i];

    if (state == LEFT) {
      delay(200);
      turnRight();
      Serial.print(state);
    }
    if (state == RIGHT) {
      delay(200); // to calibrate with time 
      turnLeft();
      Serial.print(state);
    }

    if (state == FORWARD && (ult_distance > 20 || ult_distance < -5)) {
      delay(200);
      moveForward();
      Serial.print(state);
    } else {
      moveBackward();
      Serial.print(state);
      stopped();
      delay(500);
    }
    if (state != RIGHT && state != LEFT && state != FORWARD) {
      stopped();
    }
  }
  pinMode(reverse_led, LOW);
  delay(50);
}

void moveToTarget(float targetX, float targetY) {
  const float maxDistance = 20.0;     // Maximum distance to move in one iteration
  const float targetThreshold = 1.0;  // Threshold distance to consider reaching the target

  while (true) {
    float theta = atan2(targetY - currentY, targetX - currentX);              // Angle between robot and target
    float d = sqrt(pow(targetX - currentX, 2) + pow(targetY - currentY, 2));  // Distance to target
    // Check if the target has been reached
    if (d <= targetThreshold) {
      break;
    }
    // Read sensor to detect obstacles
    readSensor();

    // Check if obstacle detected within a certain threshold distance
    if (ult_distance < maxDistance) {
      // Correct the angle by turning right
      turnRight();
    } else {
      // Calculate the proportional angle correction
      float angleError = theta - atan2(targetY - (currentY + maxDistance * sin(theta)), targetX - (currentX + maxDistance * cos(theta)));
      // Normalize the angle error to [-PI, PI]
      if (angleError > PI) {
        angleError -= 2 * PI;
      } else if (angleError < -PI) {
        angleError += 2 * PI;
      }

      // Correct the angle by turning proportional to the error
      float angleCorrection = 0.5 * angleError;  // Proportional gain
      // ===>>>> turn();
      // Move forward a maximum distance towards the target
      float moveDistance = min(maxDistance, d);  // Move as close as possible to the target
      // ====>>>> moveForward();

      // Update the current position
      currentX += moveDistance * cos(theta);
      currentY += moveDistance * sin(theta);
    }
  }
}

// THE MAIN NAVIGATION FUNCTION
// STILL IN THE EDITING
void navigation() {

  // DISPLACEMENT
  if (mode == EXPLORE) {
    Serial.println("Entering the movement");
    resetPath();
    // The robot should explore for the specified duration
    while (millis() < duration) {
      // Perform the following actions every 0.25 seconds
      if (millis() - LoopDelay >= loopPeriod) {
        readSensor();          // Read the sensor
        movement();            // Move randomly
        LoopDelay = millis();  // Update for the next check
      }
    }
    stopped();
    mode = HOME;  // Change the mode to head back home
    delay(4000);
    Serial.println("Done with the movement");
  }

  if (mode == HOME) {
    Serial.println("mode of going home");
    // Start by stopping and rotating
    stopped();
    rotation();  // Perform a 360-degree rotation
    stopped();
    printArray();
    // Return to the origin
    do {
      moveBackward();
      reverseDir();  // Reverse movement of the robot
      delay(200);
    } while (millis() < newDuration);

    stopped();
    rotation();  // Perform a 360-degree rotation
    stopped();

    mode = EXPLORE;
    Serial.println("done with reverse");
    duration = millis() + 30000UL;
  }

  if (mode == FOOD) {
    // Pursue the food target
    moveToTarget(targetX, targetY);
  } else {
    // Keep the robot stopped in other cases
    stopped();
  }
}

void loop() {

  // read the state of the button
  btnState = digitalRead(btnPin);
  state = FORWARD;

  do {

    // cheching the WiFi Module
    if (Serial.available() != 0) {
      readCoordinates();
      // change the mode if there is a target detected
      if (targetY != 0.0 || targetX != 0.0) {
        mode = FOOD;
      } else {
        mode = EXPLORE;  // remain in Exploration
      }
    } else {
      mode = EXPLORE;  // remain in exploration mode
    }
    navigation();
  } while (btnState == HIGH);
}
