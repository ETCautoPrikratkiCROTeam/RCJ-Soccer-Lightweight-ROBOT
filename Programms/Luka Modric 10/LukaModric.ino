#include <Barriers.h>
#include <Commands.h>
#include <IMUBoschBNO055.h>
#include <IRFinders.h>
#include <Motors.h>
#include <PID.h>
#include <ReflectanceSensors.h>
#include <Switches.h>
#include <VL53L0XsAnalog.h>
#include <IMUBoschBNO055Analog.h>


//Konfiguracija
#define BLUETOOTH_BPS 9600 // Bluetooth module specific. Usually 9600 or 115200.
#define SPEED_LIMIT 30 // 0 - 100. For example if you test powerful motors. 0 turns the motors off - useful for testing.
#define BRZINA 30
#define SENZORI 400
#define VRATI 500
#define ROT 2
#define treshold 400
#define LINI 400
#define LINIJAMS 200

//Objects definitions
Barriers barriers(&Serial1); // Infrared barriew, checks ball possession
Commands commands(&Serial1); // Commands for the robot.

IMUBoschBNO055Analog imu; // Compass. Reads north direction in an absolute system (o the playground). The other one is robot's (relative).
IntervalTimer timer; // Timer interrupts the main program and updates line and ball data.
IRFinders irFinders(&Serial1); // Ball sensors.
Motors motors(true, &Serial1); // Motors
PID pid(1, 2000, 0); // PID controller, regulates motors' speeds
ReflectanceSensors reflectanceSensors(0, 0.25, &Serial1); // Line sensors
Switches switches(&Serial1); // Switches, for example for starting of the robot.
VL53L0XsAnalog lidars(&Serial1);


// Robot's finite states
enum State { APPROACHING_BALL, AVOIDING_LINE, IDLE, PAUSED, POSSESSES_BALL };
State state; // Current state

// Functions' declarations
void setState(::State);
void print(String message, bool eol = false);
void go(float speed, float angle, float rotation, int speedLimit);
void motorSetSpeed(int n, int speed);
void readLineSensors();
void setupMotors();
void motorsOff();
void motorsOn();
float rotationToMaintainHeading(float headingToMaintain);

int lin[32];
bool line;

int headingToMaintain;
int green;
int red;
bool lineUp = false;
bool lineDown = false;
bool lineLeft = false;
bool lineRight = false;
bool lineUpRight = false;
bool lineUpLeft = false;
bool lineDownRight = false;
bool lineDownLeft = false;


float lowPassFilter(float newValue, float oldValue, float alpha) {
  // Alpha predstavlja faktor filtriranja, obično između 0 i 1.
  // Što je veća vrednost alpha, to će brže filter reagovati na promene.
  return alpha * newValue + (1 - alpha) * oldValue;
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  delay(300);

//Add IMU


  delay(100);
  imu.add(A2);
  delay(100);
  
  setupMotors();
  
  headingToMaintain = 0;
//Add IR ball sensors - unchecked

  pinMode(A12, INPUT);
  pinMode(A7, INPUT);
  pinMode(38,OUTPUT);
  pinMode(39,OUTPUT);
  pinMode(40,OUTPUT);
  pinMode(41,OUTPUT);
  
  //irFinders.add(A1, A2); // Angle and distance - check function declaration



//Line sensors
  
  pinMode(A10, INPUT);
  pinMode(A11, INPUT);
  pinMode(29,OUTPUT);
  pinMode(30,OUTPUT);
  pinMode(31,OUTPUT);
  pinMode(32,OUTPUT);

// switchevi
 pinMode(10, INPUT); 
 pinMode(20, INPUT);
 
 
  

  Serial.println("Da");
  setupMotors();
  
  motorsOff();
  motorsOn();
}  
/*********************************************GLAVNA PETLJA******************************************************/
void loop() {
  
  
  
  int smjer = map(analogRead(A7), 10, 850, 0, 360);
  int kompas = rotationToMaintainHeading(headingToMaintain);
  int udaljenost = map(analogRead(A3), 0,1023,0,200);
  //Serial.println(analogRead(A3));
  
  
  if (smjer>180){
    smjer=smjer-360;
    }

  float prethodnaVrijednost;
  smjer = lowPassFilter(smjer, prethodnaVrijednost, 0.7); //mijenjas faktor (zadnji broj) izmedju 0 i 1

  prethodnaVrijednost = smjer;

  
 
  
  if (digitalRead(20)==LOW){
    go(30, -180, 0, 0);
    //Serial.println(smjer);
    if(Serial1.available() > 0){
      String Kit = Serial1.readStringUntil('#');
      Serial1.clear();
      green = int(Kit[0]) - 48;
      red = int(Kit[2]) - 48;
      Serial.println(green);
    }
    motorKickerOFF();
    }
  else{
    delayMicroseconds(100);
    motorKickerON(130);
    linija2();
    if(smjer < 0){
      smjer = smjer+20;
    }
    
    Serial.println(smjer);
    if (smjer > -20 && smjer < -10){
      go(40, 0, kompas, 40);
    }
    else if (smjer>-40 && smjer<-20){
      go(BRZINA, -smjer*1.3, kompas, BRZINA);  
    }
    else if (smjer>10 && smjer<40){
      go(BRZINA, -smjer*1.2, kompas, BRZINA);  
    }
    else if (smjer>-90 && smjer<-41){
      go(30, -smjer*2, kompas, 30);
    }
    else if (smjer>41 && smjer<90){
      go(BRZINA, -smjer*2, kompas, BRZINA);
    }
    else if (smjer<-80 && smjer>-105){
      go(BRZINA, -135, kompas, BRZINA);
    }
    else if (smjer>80 && smjer<105){
      go(BRZINA, 135, kompas, BRZINA);
    }
    else if (smjer<-105 && smjer>-180){
      go(BRZINA, -170, kompas, BRZINA);
    }
    else if (smjer>105 && smjer<180){
      go(BRZINA, 170, kompas, BRZINA);
    }
    else {
      go(20, 160, kompas, 20);
    }
    
  }

}




void error(String message){

}
/** Moves the robot in order to elinimate errors (for x and y directions).
@param errorX - X axis error.
@param errorY - Y axis error.
@param headingToMaintain - Heading to maintain.
*/


/** Angle difference
@param angle1 - In degrees
@param angle2 - In degrees
@return - Difference, between -180 and 180 degrees
*/
float normalized(float angle) {
  if (angle < -180)
    angle += 360;
  else if (angle > 180)
    angle -= 360;
  return angle;
}

/** Print to 2 serial ports: Arduino monitor and Your mobile (Bluetooth).
@param message
@param eol - end of line
*/
void print(String message, bool eol) {
  if (eol) {
    Serial.println(message);
    Serial2.println(message);
  }
  else {
    Serial.print(message);
    Serial2.print(message);
  }
}

/** A rotation needed to maintain the requested heading
@return - Rotation
*/
float rotationToMaintainHeading(float headingToMaintain) {
  float kompas = map(analogRead(A2), 0, 1023, 0, 360);
  float kompi = fmod((kompas+360), 360);
  float rotation = map(normalized(headingToMaintain - kompi), -180, 180, -150, 150);
  if (rotation < 0){
    rotation = rotation* 1.2;
  }
  return rotation;
}



/** State change
@param newState - Robot's new state.
*/


/** Stop all the motors
*/
bool stop() {
  motors.go(0, 0);
  return true;
}

/** Periodic functions, invoked by timer interrupt or simply in the program's main loop.
*/
void update() {
  reflectanceSensors.anyBright();
  
}

void setupMotors() {

  // Motor 1
  pinMode(33,OUTPUT); //SLP
  pinMode(2,OUTPUT);  //PWM
  pinMode(6,OUTPUT);  //DIR

  // Motor 2
  pinMode(34,OUTPUT); //SLP
  pinMode(3,OUTPUT);  //PWM
  pinMode(7,OUTPUT);  //DIR

  // Motor 3
  pinMode(35,OUTPUT); //SLP
  pinMode(4,OUTPUT);  //PWM
  pinMode(8,OUTPUT);  //DIR

  // Motor 4
  pinMode(36,OUTPUT); //SLP
  pinMode(5,OUTPUT);  //PWM
  pinMode(9,OUTPUT);  //DIR

  //Motor 5
  pinMode(28, OUTPUT); //SLP
  pinMode(18, OUTPUT); // PWM
  pinMode(19, OUTPUT); // DIR
  

  Serial.println("Motors....OK");
  
  
}

int motorPins[5][3] = {{33,2,6},{34,3,7},{35,4,8},{36,5,9},{28,18,19}};
int reverse[4] = {0,0,0,0};
int speedLevel[4] = {0,0,0,0};

void motorsOn() {
  digitalWrite(33,HIGH);
  digitalWrite(34,HIGH);
  digitalWrite(35,HIGH);
  digitalWrite(36,HIGH);
  
}

void motorsOff() {
  digitalWrite(28,LOW);
  digitalWrite(33,LOW);
  digitalWrite(34,LOW);
  digitalWrite(35,LOW);
  digitalWrite(36,LOW);
}
void motorKickerON(int speed){
  if(speed > 0){
    digitalWrite(28,HIGH);
    digitalWrite(19,HIGH);
    analogWrite(18,speed);
  }
  else{
    digitalWrite(28,HIGH);
    digitalWrite(19,LOW);
    analogWrite(18,speed);
  }
  
  
  
}
void motorKickerOFF(){
  digitalWrite(28,HIGH);
  
}

void motorSetSpeed(int n, int speed) {
  speed = int(speed * 2.55);
  if (speed < 0) {
    digitalWrite(motorPins[n][2], 1 - reverse[n]);
  } else {
    digitalWrite(motorPins[n][2], 0 + reverse[n]);
  }
  analogWrite(motorPins[n][1], abs(speed) - int((speedLevel[n]*abs(speed)/10)));
}


void go(float speed, float angle, float rotation, int speedLimit = 10) {
  
  angle += 135;
  angle *= -1;
  angle = toRad(angle);
  float si = sin(angle);
  float co = cos(angle);
  float motorSpeed[4];
  
  motorSpeed[0] = -speed * si - rotation;
  motorSpeed[1] = -speed * co - rotation;
  motorSpeed[2] =  speed * si - rotation;
  motorSpeed[3] =  speed * co - rotation;

  float maxMotorSpeed = abs(motorSpeed[0]);
  for (int i = 0; i < 4; i++) {
    if (abs(motorSpeed[i]) > maxMotorSpeed) 
      maxMotorSpeed = abs(motorSpeed[i]);
  }
  for (int i = 0; i < 4; i++) {
   /* Serial.print(int(motorSpeed[i]*2.55));
    Serial.print("  ");
    Serial.print(speedLimit);
    Serial.print("  ");
    Serial.println(maxMotorSpeed);
    */
    if (speedLimit == 0)
      motorSetSpeed(i, 0);
    else if (maxMotorSpeed > speedLimit)
      motorSetSpeed(i, motorSpeed[i] / maxMotorSpeed * speedLimit);
    else
      motorSetSpeed(i, motorSpeed[i]);
  }
}

void readLineSensors() {

  int counter = 0;
  bool found = false;
  
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      for (int k = 0; k < 2; k++) {
        for (int l = 0; l < 2; l++) {
          
          digitalWrite(29,i);
          digitalWrite(30,j);
          digitalWrite(31,k);
          digitalWrite(32,l);

          delay(1);

          lin[counter] = analogRead(A10);
          lin[counter + 16] = analogRead(A11);
          Serial.print(lin[counter + 16]);
          Serial.print("  ");
          //Serial.print(lin[counter + 16]);
          //Serial.print("   ");
          if (lin[counter] > treshold || lin[counter + 16] > treshold) found = true;
          counter++;
          
          
          
        }
      }
    }
  }
  if (lin[4]> LINI || lin[15]> LINI || lin[6]> LINI || lin[13]> LINI || lin[7]> LINI || lin[12]> LINI || lin[14]> LINI || lin[5]> LINI){
    lineUp = true;
    }
  else{
    lineUp = false;
    }
  if (lin[0]> LINI || lin[2]> LINI || lin[8]> LINI || lin[10]> LINI || lin[1]> LINI || lin[3]> LINI || lin[9]> LINI || lin[11]> LINI){
    lineLeft = true;
    }
  else{
    lineLeft = false;
    }
  if (lin[16]> LINI || lin[18]> LINI || lin[24]> LINI || lin[26]> LINI || lin[17]> LINI || lin[19]> LINI || lin[25]> LINI || lin[27]> LINI){
    lineRight = true;
    }
  else{
    lineRight = false;
    }
  if (lin[20]> LINI || lin[22]> LINI || lin[28]> LINI || lin[30]> LINI || lin[21]> LINI || lin[23]> LINI || lin[29]> LINI || lin[31]> LINI){
    lineDown = true;
    }
  else{
    lineDown = false;
    }
  // Additional combinations
lineUpRight = lineUp && lineRight;
lineUpLeft = lineUp && lineLeft;
lineDownRight = lineDown && lineRight;
lineDownLeft = lineDown && lineLeft;
  Serial.println();
  if (found) {
    line = true;
  } else {
    line = false;
  }

}
