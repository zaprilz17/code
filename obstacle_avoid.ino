const int trigPin = 1;
const int echoPin = 2;
const int in1 = 3;
const int in2 = 11;
const int in3 = 12;
const int in4 = 13;
const int obstacleDistance = 20;
void setup() {
pinMode(trigPin, OUTPUT);
pinMode(echoPin, INPUT);
pinMode(in1, OUTPUT);
pinMode(in2, OUTPUT);
pinMode(in3, OUTPUT);
pinMode(in4, OUTPUT);
Serial.begin(9600);
}
void loop() {
long distance = measureDistance();
Serial.print("Distance: ");
Serial.print(distance);
Serial.println(" cm");
if (distance < obstacleDistance) {
stopCar();
delay(500);
turnRight();
delay(800);
}
else {
11
moveForward();
}
}
long measureDistance() {
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
long duration = pulseIn(echoPin, HIGH);
long distance = (duration * 0.0343) / 2;
return distance;
}
void moveForward() {
digitalWrite(in1, HIGH);
digitalWrite(in2, LOW);
digitalWrite(in3, HIGH);
digitalWrite(in4, LOW);
}
void turnRight() {
digitalWrite(in1, HIGH);
digitalWrite(in2, LOW);
digitalWrite(in3, LOW);
digitalWrite(in4, HIGH);
}
void stopCar() {
digitalWrite(in1, LOW);
digitalWrite(in2, LOW);
digitalWrite(in3, LOW);
digitalWrite(in4, LOW);
}
