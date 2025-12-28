#include <Wire.h>
#include <LiquidCrystal.h>

// ===== LCD =====
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// ===== Motor Pins =====
int ENA = 3;      
int IN1 = 12;    
int IN2 = 13;    

int ENB = 11;     
int IN3 = 2;      
int IN4 = A1;     

// ===== PWM =====
int basePWM   = 210;
int uphillPWM = 240;
int rotatePWM = 150;
int maxPWM    = 255;
int minPWM    = 50;

// ===== MPU6050 =====
uint8_t MPU_ADDR = 0x68;
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define GYRO_ZOUT_H  0x47


enum CarState {MOVING, TOP_WAIT, ROTATE, STOPPED};
CarState carState = MOVING;


float maxPitch = 0;
unsigned long climbStartTime = 0;
unsigned long waitStartTime  = 0;
bool reachedTop = false;


float yawAngle = 0;
unsigned long lastGyroTime = 0;


int16_t readMPU16(uint8_t reg){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(MPU_ADDR, (uint8_t)2);
  return Wire.read() << 8 | Wire.read();
}

float readPitch(){
  int16_t ax = readMPU16(ACCEL_XOUT_H);
  int16_t ay = readMPU16(ACCEL_YOUT_H);
  int16_t az = readMPU16(ACCEL_ZOUT_H);
  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;
  return atan2(ax_g, sqrt(ay_g*ay_g + az_g*az_g)) * 180.0 / PI;
}

float readRoll(){
  int16_t ax = readMPU16(ACCEL_XOUT_H);
  int16_t ay = readMPU16(ACCEL_YOUT_H);
  int16_t az = readMPU16(ACCEL_ZOUT_H);
  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;
  return atan2(ay_g, sqrt(ax_g*ax_g + az_g*az_g)) * 180.0 / PI;
}

float readGyroZ(){
  int16_t gz = readMPU16(GYRO_ZOUT_H);
  return gz / 131.0;
}


void setMotors(int leftPWM, int rightPWM){
  leftPWM  = constrain(leftPWM, 0, 255);
  rightPWM = constrain(rightPWM, 0, 255);

  // 左电机前进
  digitalWrite(IN1, HIGH); 
  digitalWrite(IN2, LOW);
  analogWrite(ENA, leftPWM);

  // 右电机前进
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, rightPWM);
}

void stopCar(){
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// ===== Setup =====
void setup(){
  Wire.begin();
  lcd.begin(16,2);
  Serial.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0);
  Wire.endTransmission();

  lcd.print("Car Ready");
  delay(1000);
  lcd.clear();
}

// ===== Loop =====
void loop(){
  float pitch = readPitch();
  float roll  = readRoll();

  if(carState == MOVING){
    lcd.setCursor(0,0);
    lcd.print("Pitch:");
    lcd.print(pitch,1);
    lcd.print("   ");
  }

  int leftPWM = basePWM;
  int rightPWM = basePWM;

  if(pitch > 5.0){
    leftPWM  = uphillPWM;
    rightPWM = uphillPWM;
  }

  if(roll > 2.0){ leftPWM += 20; rightPWM -= 20; }
  else if(roll < -2.0){ leftPWM -= 20; rightPWM += 20; }

  if(pitch >= 18.0 && pitch <= 30.0){
    if(pitch > maxPitch) maxPitch = pitch;
  }

  switch(carState){

    case MOVING:
      setMotors(leftPWM, rightPWM);

      if(pitch > 15.0 && climbStartTime == 0)
        climbStartTime = millis();

      if(maxPitch>=18.0 && maxPitch<=30.0 &&
         pitch < 6.0 &&
        (millis()-climbStartTime)>2000 &&
         !reachedTop)
      {
        reachedTop = true;
        carState = TOP_WAIT;
        waitStartTime = millis();
        stopCar();

        lcd.setCursor(0,1);
        lcd.print("Max Angle:");
        lcd.print(maxPitch,1);
      }
      break;

    case TOP_WAIT:
      stopCar();
      if(millis() - waitStartTime >= 4000){
        carState = ROTATE;
        yawAngle = 0;
        lastGyroTime = millis();
      }
      break;

    case ROTATE:{
      unsigned long now = millis();
      float dt = (now - lastGyroTime) / 1000.0;
      lastGyroTime = now;
      yawAngle += abs(readGyroZ()) * dt;

      
      digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
      analogWrite(ENA, rotatePWM);

      digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
      analogWrite(ENB, rotatePWM);

      if(yawAngle >= 360){
        stopCar();
        carState = STOPPED;
      }
    }
    break;

    case STOPPED:
      stopCar();
      lcd.setCursor(0,1);
      lcd.print("Max Angle:");
      lcd.print(maxPitch,1);
      while(1);
      break;
  }

  delay(50);
}p
