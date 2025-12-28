char t;
#define IN1 3
#define IN2 11
#define IN3 12
#define IN4 13
#define LED 9
void setup() {
pinMode(IN1, OUTPUT);
pinMode(IN2, OUTPUT);
pinMode(IN3, OUTPUT);
pinMode(IN4, OUTPUT);
pinMode(LED, OUTPUT);
Serial.begin(9600);
}
void stopCar() {
digitalWrite(IN1, LOW);
digitalWrite(IN2, LOW);
digitalWrite(IN3, LOW);
digitalWrite(IN4, LOW);
}
void forward() {
stopCar();
digitalWrite(IN1, HIGH);
digitalWrite(IN3, HIGH);
}
13
void backward() {
stopCar();
digitalWrite(IN2, HIGH);
digitalWrite(IN4, HIGH);
}
void turnRight() {
stopCar();
digitalWrite(IN3, HIGH);
}
void turnLeft() {
stopCar();
digitalWrite(IN1, HIGH);
}
void loop() {
if (Serial.available()) {
t = Serial.read();
}
if (t == 'F') {
forward();
}
else if (t == 'B') {
backward();
}
else if (t == 'L') {
turnLeft();
}
else if (t == 'R') {
turnRight();
}
else if (t == 'S') {
stopCar();
}
else if (t == 'W') {
digitalWrite(LED, HIGH);
}
else if (t == 'w') {
digitalWrite(LED, LOW);
}
14
delay(20);
}