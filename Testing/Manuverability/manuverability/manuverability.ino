#define enA 5 //Brown
#define enB 6 //White
#define in3 4 //Green
#define in4 7 //Purple
#define in1 3 //Red
#define in2 2 //Ornage
#define button 4
#define lSpeed 250
#define rSpeed 250
#define lSpeedBack 250
#define rSpeedBack 250
#define lSpeedTurn 80
#define rSpeedTurn 80

const int trigPin = 12;
const int echoPin = 13;
int us1Dist;
long duration;
int distance;

void setup() {
  // put your setup code here, to run once:

  //Motor Setup
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  //TURN
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, lSpeedTurn); // Send PWM signal to L298N Enable pin
  analogWrite(enB, rSpeedTurn);
  delay(1075);

  //STOP
  analogWrite(enA, 0); // Send PWM signal to L298N Enable pin
  analogWrite(enB, 0);
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:

}