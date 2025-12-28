#include <LiquidCrystal.h>
#define LEFT_SENSOR  A2
#define RIGHT_SENSOR A3
#define BLACK_THRESHOLD 200

#define MOTOR_LEFT_SPEED     3
#define MOTOR_LEFT_FORWARD  12
#define MOTOR_LEFT_BACKWARD 13
#define MOTOR_RIGHT_SPEED   11
#define MOTOR_RIGHT_FORWARD 2
#define MOTOR_RIGHT_BACKWARD A1

//rs=8, en=9, d4=4, d5=5, d6=6, d7=7
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);


const float SPEED_CM_PER_MS = 2.13 / 9 / 1000 * 100; 
const float STOP_DISTANCE = 60.0; 
const unsigned long STOP_DURATION = 3000; 
const int forwardSpeed = 90;     

// ---------------- Motor control ----------------
void motor(int leftPWM, int rightPWM){
  if(leftPWM >= 0){
    digitalWrite(MOTOR_LEFT_FORWARD, HIGH);
    digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
  }else{
    digitalWrite(MOTOR_LEFT_FORWARD, LOW);
    digitalWrite(MOTOR_LEFT_BACKWARD, HIGH);
    leftPWM = -leftPWM;
  }
  analogWrite(MOTOR_LEFT_SPEED, leftPWM);

  if(rightPWM >= 0){
    digitalWrite(MOTOR_RIGHT_FORWARD, HIGH);
    digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
  }else{
    digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
    digitalWrite(MOTOR_RIGHT_BACKWARD, HIGH);
    rightPWM = -rightPWM;
  }
  analogWrite(MOTOR_RIGHT_SPEED, rightPWM);
}

void stopCar(){
  motor(0,0);
}

// ---------------- Global variables ----------------
int turning = 0;               // 0 = forward, 1 = pivot right, 2 = pivot left
unsigned long turnStart = 0;

// Speeds
const int pivotSpeedMax = 180;   // fast pivot for sharp corner
const int pivotSpeedMin = 120;   // maintain enough speed for pivot precision
const unsigned long pivotTimeMax = 800;  // time to reduce pivot speed


unsigned long runStartTime = 0;  
float totalDistance = 0.0;       
bool isFirst60cm = true;         
bool isStoppedAt60cm = false;    
unsigned long stopStartTime = 0; 

// ---------------- Setup ----------------
void setup(){
  pinMode(MOTOR_LEFT_FORWARD,  OUTPUT);
  pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_BACKWARD,OUTPUT);

  pinMode(LEFT_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);

  
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Distance: 0.0cm");
  lcd.setCursor(0, 1);
  lcd.print("Time: 0.0s");

  stopCar();
  runStartTime = millis(); 
}

// ---------------- Loop ----------------
void loop(){
  unsigned long now = millis();
  
  unsigned long runTimeMs = now - runStartTime;
  float runTimeS = runTimeMs / 1000.0;
  
  
  if(!isStoppedAt60cm){
    totalDistance = SPEED_CM_PER_MS * runTimeMs;
  }

  
  if(totalDistance >= STOP_DISTANCE && isFirst60cm && !isStoppedAt60cm){
    isStoppedAt60cm = true;
    stopStartTime = now;
    stopCar(); 
    lcd.clear();
    lcd.print("Stopped at 60cm");
    lcd.setCursor(0, 1);
    lcd.print("Wait 3s...");
  }
  
  if(isStoppedAt60cm && (now - stopStartTime) >= STOP_DURATION){
    isStoppedAt60cm = false;
    isFirst60cm = false; 
    runStartTime = millis(); 
    totalDistance = STOP_DISTANCE; 
  }

 
  if(!isStoppedAt60cm){
    lcd.setCursor(0, 0);
    lcd.print("Distance: ");
    lcd.print(totalDistance, 1); 
    lcd.print("cm");
    lcd.setCursor(0, 1);
    lcd.print("Time: ");
    lcd.print(runTimeS, 1); 
    lcd.print("s");
  }

  
  if(!isStoppedAt60cm){
    int L = analogRead(LEFT_SENSOR);
    int R = analogRead(RIGHT_SENSOR);

    bool leftBlack  = (L > BLACK_THRESHOLD);
    bool rightBlack = (R > BLACK_THRESHOLD);

    switch(turning){
      case 0: // Forward mode
        if(leftBlack && !rightBlack){
          turning = 1;      // Start pivot right
          turnStart = now;
        }
        else if(!leftBlack && rightBlack){
          turning = 2;      // Start pivot left
          turnStart = now;
        }
        else if(!leftBlack && !rightBlack){
          motor(forwardSpeed, forwardSpeed);   
        }
        else if(leftBlack && rightBlack){
          stopCar();        // Both black → stop
        }
        break;

      case 1: // Pivot right
        {
          unsigned long pivotTime = now - turnStart;
          int speed = pivotSpeedMax - (pivotSpeedMax - pivotSpeedMin) * pivotTime / pivotTimeMax;
          if(speed < pivotSpeedMin) speed = pivotSpeedMin;
          motor(-speed, speed);  // Pivot right
          if(!leftBlack || pivotTime > 1500) turning = 0; // Resume forward
        }
        break;

      case 2: // Pivot left
        {
          unsigned long pivotTime = now - turnStart;
          int speed = pivotSpeedMax - (pivotSpeedMax - pivotSpeedMin) * pivotTime / pivotTimeMax;
          if(speed < pivotSpeedMin) speed = pivotSpeedMin;
          motor(speed, -speed);  // Pivot left
          if(!rightBlack || pivotTime > 1500) turning = 0; // Resume forward
        }
        break;
    }
  }
}