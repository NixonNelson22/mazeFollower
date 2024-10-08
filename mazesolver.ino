// assuming a 6x6 or 7x7 grid / 154 or 196 so safe 600
//Line following code
// Motor control pins for MX1508
const int PWMA1 = 6;
const int PWMA2 = 9;
const int PWMB1 = 11;
const int PWMB2 = 10;

// Define the pins for TCRT5000 sensors
const int sensorPins[5] = {A0, A1, A2, A3, A6};
int sensorAnalogValues[5];
int sensorDigitalValues[5];
int threshold = 500;  // Threshold for detecting black line
const int leftmostsensor = A6;
const int rightmostsensor = A0;
const int centersensor = A2;

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

  pinMode(PWMA1, OUTPUT);
  pinMode(PWMA2, OUTPUT);
  pinMode(PWMB1, OUTPUT);
  pinMode(PWMB2, OUTPUT);

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
  // mapping 
  //// get encoder data and approximate the values to a 2d matrix
  //// landmarks are +,L,O,# sections
  //// straight | sections
  //// goal is # square 
  //// has to be stored in memory
  // localization 
  //// get data from mapping and check if we are in correct place
  //// path error correction
  
  // shortest path algo
  //// find the shortest path from mapping


  get_Input();
  floodfill();
  // start polling data from line sensor

  // check if bot is in goal

  // check if no line found  
  
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
    analogWrite(PWMB1, right);
  } else {
    analogWrite(PWMB2, -right);
  }
  if (left > 0) {
    analogWrite(PWMA1, left);
  } else {
    analogWrite(PWMA2, -left);
  }
}

//mapping 

void symbolDetected(){
  for (int i = 0; i < 6; i++) {
    sensorDigitalValues[i] = analogRead(sensorPins[i]) < threshold ? 0 : 1;

	// check which landmark reached in which orientation 
	//  if rightmostdetects and left most doesent{
	//     the landmark is L with one adjacent node 
	//     and turns to the right 
	//      __.
	//        |
	//    myhashtable.put(1,"LR");
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
}
void floodfill(){
	// start
	// stack node distance visited dir
	// T and + is a branch node 
	// # switches modes when in short path mode
	// initial value 
	int head[50]; 
	char node[600] ;
	float distance[600];
	bool visited[600];
	char dir[600];

  node[0]= 'o';
  distance[0]= 0.0;
  visited[0]= true;
  dir[0]= ' ';
  head[0]= 0;

  robot_control('F'); // first move 
  i = 1;
  if(symbolDetected())
	  robot_control('brake');
	  while(){
		// record node
		node[i++] = detected_symbol();
		distance[i++] = speedsensor();
		dir[i++] = mpu();
		visited[i++] = true;
		
		if

	  }

  
}
