#include "MeMCore.h"
#define ULTRASONIC 12
#define ULTRASONIC_TIMEOUT 2000
#define SPEED_OF_SOUND 340

MeLineFollower lineFinder(PORT_2);
MeDCMotor leftMotor(M2);
MeDCMotor rightMotor(M1);
MeRGBLed led(7, 2);
MeBuzzer buzzer;

// chirp like a robot heheh
void chirp() {
  for (int i = 0; i < 5; i++) {
    buzzer.tone(1500 + (i * 200), 40);
    led.setColor(255, 0, 0);
    led.show();
    delay(50);
  }

  for (int i = 0; i < 8; i++) {
    buzzer.tone(2500 - (i * 150), 30);
    led.setColor(0, 255, 0);
    led.show();
    delay(40);
  }

  buzzer.tone(1800, 100);
  led.setColor(0, 0, 255);
  led.show();
  delay(130);
  buzzer.tone(2200, 80);
  led.setColor(255, 165, 0);
  led.show();
  delay(100);

  for (int i = 0; i < 15; i++) {
    int randomPitch = random(1000, 3000);
    int randomDuration = random(20, 60);

    int randomColorR = random(50, 255);
    int randomColorG = random(50, 255);
    int randomColorB = random(50, 255);

    buzzer.tone(randomPitch, randomDuration);

    led.setColor(randomColorR, randomColorG, randomColorB);
    led.show();
    delay(randomDuration + 10);
  }
}

void runMotors(const int leftSpeed, const int rightSpeed) {
  leftMotor.run(-leftSpeed);
  rightMotor.run(rightSpeed);
}

float readUltrasonic() {
  pinMode(ULTRASONIC, OUTPUT);
  digitalWrite(ULTRASONIC, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC, LOW);
  pinMode(ULTRASONIC, INPUT);
  long duration = pulseIn(ULTRASONIC, HIGH, ULTRASONIC_TIMEOUT);
  if (duration > 0) {
    return (duration / 2.0 / 1000000.0f * SPEED_OF_SOUND * 100);
  }
  return 5000;
}

float readIR() {
  //turn off IR emitter (also turns on red light due to how its wired, but should not affect functionality)
  digitalWrite(A2, LOW);
  digitalWrite(A3, HIGH);
  //wait for saturation
  delayMicroseconds(500);
  //measure ambient
  float ambient = analogRead(A0);
  // Serial.println("ambient: " + String(ambient));

  //turn on IR emitter
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  //wait for saturation
  delayMicroseconds(500);
  float reading = analogRead(A0);
  // Serial.println("reading: " + String(reading));

  //return final reading which is the ambient - reading
  // Serial.println("final: " + String(ambient - reading));
  return (ambient - reading);
}

////////
//PID
///////
int baseTurnSpeed = 215;
int PIDConstrain = 255 - baseTurnSpeed;

unsigned long currentTime = 0, previousTime = 0;
// Global variables for Ultrasonic PID memory
double integralUS = 0, previousErrorUS = 0;

double PID_Ultrasonic(float input, float target) {
  double Kp = 240, Ki = 0.1, Kd = 2;
  currentTime = millis();
  double elapsed = (double)(currentTime - previousTime) / 1000.0;
  if (elapsed <= 0) elapsed = 0.001;

  double errorUS = target - input;
  integralUS += errorUS * elapsed;
  double derivativeUS = (errorUS - previousErrorUS) / elapsed;

  previousErrorUS = errorUS;
  double output = (errorUS * Kp) + (integralUS * Ki) + (derivativeUS * Kd);
  // Serial.println("output: " + String(output));
  // Serial.println("output: " + String(constrain(output, -PIDConstrain, PIDConstrain)));
  return output;
}

// Global variables for IR PID memory
double integralIR = 0, previousErrorIR = 0;

double PID_IR(float input, float target) {
  double Kp = 240/100, Ki = 0.1/100, Kd = 2/100;

  currentTime = millis();
  double elapsed = (double)(currentTime - previousTime) / 1000.0;
  if (elapsed <= 0) elapsed = 0.001;

  double errorIR = input - target;

  integralIR += errorIR * elapsed;
  double derivativeIR = (errorIR - previousErrorIR) / elapsed;

  previousErrorIR = errorIR;

  double output = (errorIR * Kp) + (integralIR * Ki) + (derivativeIR * Kd);
  // Serial.println("output: " + String(output));
  // Serial.println("output: " + String(constrain(output, -PIDConstrain, PIDConstrain)));
  return output;
}

// measured average values of black and white
int colourL[3] = { 35, 84, 61 };
int colourH[3] = { 189, 351, 231 };

void colourSensor(int* returnList, bool raw = false) {

  // Red
  digitalWrite(A3, LOW);
  digitalWrite(A2, HIGH);
  delay(20);
  int red = analogRead(A1);
  digitalWrite(A3, LOW);
  digitalWrite(A2, LOW);

  // Green
  digitalWrite(A3, HIGH);
  digitalWrite(A2, HIGH);
  delay(20);
  int green = analogRead(A1);
  digitalWrite(A3, LOW);
  digitalWrite(A2, LOW);


  // Blue
  digitalWrite(A3, HIGH);
  digitalWrite(A2, LOW);
  delay(20);
  int blue = analogRead(A1);
  digitalWrite(A3, LOW);
  digitalWrite(A2, LOW);


  // Fill the array that was passed in
  if (raw) {
    returnList[0] = red;
    returnList[1] = green;
    returnList[2] = blue;
  } else {
    returnList[0] = constrain(map(red, colourL[0], colourH[0], 0, 255), 0, 255);
    returnList[1] = constrain(map(green, colourL[1], colourH[1], 0, 255), 0, 255);
    returnList[2] = constrain(map(blue, colourL[2], colourH[2], 0, 255), 0, 255);
  }
  // Serial.println(String(returnList[0]) + " " + String(returnList[1]) + " " + String(returnList[2]));
}

// measures colour 5 times
// used to get average values of colours for pre-callibration
void calib(int white[3], int black[3]) {
  while (analogRead(A7) > 100) delay(50);  // Wait for button
  buzzer.tone(784, 200);
  white[0] = 0;
  white[1] = 0;
  white[2] = 0;
  // Serial.println("Calibrating white");
  for (int i = 0; i < 5; i++) {
    int colour[3] = { 0, 0, 0 };
    colourSensor(colour, true);
    white[0] += colour[0];
    white[1] += colour[1];
    white[2] += colour[2];
  }
  white[0] /= 5;
  white[1] /= 5;
  white[2] /= 5;
  buzzer.tone(392, 200);
  delay(100);
  while (analogRead(A7) > 100) delay(50);  // Wait for button
  buzzer.tone(784, 200);
  black[0] = 0;
  black[1] = 0;
  black[2] = 0;
  // Serial.println("Calibrating black");
  for (int i = 0; i < 5; i++) {
    int colour[3] = { 0, 0, 0 };
    colourSensor(colour, true);
    black[0] += colour[0];
    black[1] += colour[1];
    black[2] += colour[2];
  }
  black[0] /= 5;
  black[1] /= 5;
  black[2] /= 5;
  buzzer.tone(392, 200);
}

// Colour dataset
int redDataset[3] = { 200, 50, 50  };
int pinkDataset[3] = { 202, 181, 183 };
int greenDataset[3] = { 28, 163, 112 };
int blueDataset[3] = { 12, 143, 193 };
int orangeDataset[3] = { 200, 65, 50 };
int whiteDataset[3] = { 197, 242, 249 };

// vars for KNN calculation
float shortestDistance = 999999.0;
String bestColourLabel = "Unknown";

void checkColourCategory(int newR, int newG, int newB, int dataset[3], const String label) {
  long deltaR = (long)(newR - dataset[0]);
  long deltaG = (long)(newG - dataset[1]);
  long deltaB = (long)(newB - dataset[2]);

  float currentDistance = sqrt((deltaR * deltaR) + (deltaG * deltaG) + (deltaB * deltaB));

  if (currentDistance < shortestDistance) {
    shortestDistance = currentDistance;
    bestColourLabel = label;
  }
}

const String predictColour(int newR, int newG, int newB) {
  // Reset tracking vars
  shortestDistance = 999999.0;
  bestColourLabel = "Unknown";

  // Compare reading to each of the datasets
  checkColourCategory(newR, newG, newB, redDataset, "Red");
  checkColourCategory(newR, newG, newB, pinkDataset, "Pink");
  checkColourCategory(newR, newG, newB, greenDataset, "Green");
  checkColourCategory(newR, newG, newB, blueDataset, "Blue");
  checkColourCategory(newR, newG, newB, orangeDataset, "Orange");
  checkColourCategory(newR, newG, newB, whiteDataset, "White");

  // Return the name of the winner
  // for debug purposes
  if (bestColourLabel == "Red") {
    led.setColor(255, 0, 0);
  } else if (bestColourLabel == "Green") {
    led.setColor(0, 255, 0);
  } else if (bestColourLabel == "Blue") {
    led.setColor(0, 0, 255);
  } else if (bestColourLabel == "White") {
    led.setColor(255, 255, 255);
  } else if (bestColourLabel == "Orange") {
    led.setColor(255, 165, 0);
  } else if (bestColourLabel == "Pink") {
    led.setColor(255, 20, 147);
  }
  led.show();
  return bestColourLabel;
}

void moveByColour() {
  int colour[3] = { 0, 0, 0 };
  colourSensor(colour);
  String currentColour = predictColour(colour[0], colour[1], colour[2]);
  if (currentColour == "Red") {
    runMotors(0, 0);
    runMotors(-255, 255);
    delay(340);
    runMotors(0, 0);
  }

  else if (currentColour == "Green") {
    runMotors(0, 0);
    runMotors(255, -255);
    delay(340);
    runMotors(0, 0);
  }

  else if (currentColour == "Orange") {
    runMotors(0, 0);
    runMotors(-255, 255);
    delay(630);
    runMotors(0, 0);
  }

  else if (currentColour == "Pink") {
    runMotors(0, 0);
    runMotors(-255, 255);
    delay(350);
    runMotors(255, 255);
    delay(765);
    runMotors(-255, 255);
    delay(340);
    runMotors(0, 0);
  }

  else if (currentColour == "Blue") {
    runMotors(0, 0);
    runMotors(255, -255);
    delay(330);
    runMotors(255, 255);
    delay(700);
    runMotors(255, -255);
    delay(400);
    runMotors(0, 0);
  }

  else if (currentColour == "White") {
    runMotors(0, 0);
    while (true) chirp();
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(A7, INPUT);   // Button
  pinMode(A2, OUTPUT);  // A
  pinMode(A3, OUTPUT);  // B
  pinMode(A0, INPUT);   // Ir voltage
  
  // Serial.println("Setup complete");

  // // code for finding colour maually
  // // was used for pre-callibration
  // int white[3];
  // int black[3];

  // calib(white, black);
  // colourH[0] = white[0];
  // colourH[1] = white[1];
  // colourH[2] = white[2];
  // colourL[0] = black[0];
  // colourL[1] = black[1];
  // colourL[2] = black[2];
}

void loop() {

  while (analogRead(A7) > 100) delay(50);  // Wait for button
  delay(1000); //for sake of not knocking the bot around
  currentTime = millis();
  previousTime = currentTime;

  while (true) {
    float rightUS = readUltrasonic();
    float leftIR = readIR();
    bool black = lineFinder.readSensors() != S1_OUT_S2_OUT;

    if (black) {
      runMotors(0, 0);
      moveByColour();
      // Reset time for PID controllers
      currentTime = millis();
      previousTime = currentTime;
    }

    else if (rightUS < 6) {
      if (rightUS != 5000) {
        currentTime = millis();

        float PID = PID_Ultrasonic(rightUS, 6.0f);
        float right = constrain(baseTurnSpeed + PID, -255, 255);

        runMotors(100, right);
        
        previousTime = currentTime;
      }
    }

    else if (leftIR > 480) {
      currentTime = millis();

      float PID = PID_IR(leftIR, 480.0f);
      float left = constrain(baseTurnSpeed + PID, -255, 255);

      runMotors(left, 100);

      previousTime = currentTime;
    }

    else {
      // reset integral variables to prevent jumping and excessive wind up of error summation
      integralUS = 0;
      previousErrorUS = 0;
      integralIR = 0;
      previousErrorIR = 0;
      previousTime = millis();

      // Notice how every wall on the right of a colour paper is missing? (one exception is the map at the far right [pun intended])
      // rightUS has a max value of 19cm with walls on both sides. Thus, 20cm and above means there is no wall on the right
      // When there is no wall to the right, the current cell has a coloured paper.
      // Thus, slow down speed to fastest speed where it can stop after detecting black without hitting the wall

      if (rightUS > 20) {
        runMotors(255, 255);  // run speed slower in order to make it detect the black strip and stop without hitting the wall
      } else {
        runMotors(255, 255);
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////