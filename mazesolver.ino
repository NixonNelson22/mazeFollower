
#include<Hashtable.h>
//Line following code
// Motor control pins
const int AIN1 = 7;
const int AIN2 = 8;
const int PWMA = 6;
const int BIN1 = 9;
const int BIN2 = 10;
const int PWMB = 11;
const int STBY = 12;  // Define STBY pin

// Define the pins for TCRT5000 sensors
const int sensorPins[6] = {A1, A2, A3, A4, A5, A6};
int sensorAnalogValues[6];
int sensorDigitalValues[6];
int threshold = 500;  // Threshold for detecting black line

// PID control variables
float Kp = 0.7;
float Ki = 0;
float Kd = 0.5;

float Pvalue;
float Ivalue;
float Dvalue;

int previousError = 0;
int P, D, I, error;
int lsp, rsp;
int lfspeed = 230;

void setup() {
  // Initialize motor control pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  // Disable standby mode
  digitalWrite(STBY, HIGH);

  // Initialize sensor pins
  for (int i = 0; i < 6; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  Serial.begin(9600);
  delay(1000);
}

void loop() {
  // mobility script
  //// controlled by localization and path algo
  robot_control();
  // mapping 
  //// get encoder data and approximate the values to a 2d matrix
  //// landmarks are +,L,O,# sections
  //// straight | sections
  //// goal is # square 
  //// has to be stored in memory
  get_Input();
  
  // localization 
  //// get data from mapping and check if we are in correct place
  //// path error correction
  
  // shortest path algo
  //// find the shortest path from mapping
  
}

void robot_control() {
  // Read sensor values
  int weightedSum = 0;
  int sum = 0;

  for (int i = 0; i < 6; i++) {
    sensorAnalogValues[i] = analogRead(sensorPins[i]);
    int weight = (i - 2) * 1000;
    if (sensorAnalogValues[i] < threshold) {  // Sensor detects black line
      weightedSum += weight;
      sum += 1000;  // Sum up weights where the line is detected
    }
  }

  if (sum > 0) {
    error = weightedSum / sum;
  } else {
    // If the robot loses the line, use previous error to decide the direction
    if (previousError > 0) {
      motor_drive(-230, 230);  // Turn left
    } else {
      motor_drive(230, -230);  // Turn right
    }
    return;
  }

  PID_Linefollow(error);
}

void PID_Linefollow(int error) {
  P = error;
  I = I + error;
  D = error - previousError;
  
  Pvalue = Kp * P;
  Ivalue = Ki * I;
  Dvalue = Kd * D;

  float PIDvalue = Pvalue + Ivalue + Dvalue;
  previousError = error;

  lsp = lfspeed - PIDvalue;
  rsp = lfspeed + PIDvalue;

  lsp = constrain(lsp, -255, 255);
  rsp = constrain(rsp, -255, 255);

  motor_drive(lsp, rsp);
}

void motor_drive(int left, int right) {
  if (right > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, right);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, -right);
  }

  if (left > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, left);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, -left);
  }
}

//mapping 

void get_Input(){
  for (int i = 0; i < 6; i++) {
    sensorDigitalValues[i] = analogRead(sensorPins[i]) < threshold ? 0 : 1;
  }

//  hashmap 
//  key : value 
    Hashtable<String, int> myhashtable;

  // check which landmark reached in which orientation 
//  if rightmostdetects and left most doesent{
//     the landmark is L with one adjacent node 
//     and turns to the right 
//      __.
//        |
     myhashtable.put(1,"LR");
//    }
//  if rightmostdoesnt and leftmostdetects{
//      the landmark is L with one adjacent node 
//      and turns to the left
//      .__
//      |
//    put(i+1,LL)
//    }
//  if rightmostdetects and leftmostdetects or rightmostdetects and left most doesnt and line in front after delay or rightmostdoesent and leftmostdetects and line in front after {
//     the landmark is T with 2 adjacent nodes
//     can turn 2 ways
//      --| |-- ``:`` _|_
//    put(i+1,T*)
//    }
//  if rightmostdetects and leftmostdetects and line in front after delay {
//       the landmark is a +
//    put(i+1,+)
//    }
//  if all sensors detect for some delay {
//    the goal is reached landmark is #
//    put(i+1,#)
//    }
//  if front sensor detects no line{
//     the lankdmark is / dead end
//    put(i+1,/)
//    }
//



}
