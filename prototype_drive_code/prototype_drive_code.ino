#define enA 9
#define enB 5
#define in3 12
#define in4 8
#define in1 6
#define in2 7
#define button 4

void setup() {
  // put your setup code here, to run once:
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  //GO FORWARD
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, 100); // Send PWM signal to L298N Enable pin
  analogWrite(enB, 200);
  delay(2000);

  //STOP
  analogWrite(enA, 0); // Send PWM signal to L298N Enable pin
  analogWrite(enB, 0);
  delay(2000);


  //GO BACKWARD
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, 100); // Send PWM signal to L298N Enable pin
  analogWrite(enB, 200);
  delay(2000);

  //STOP
  analogWrite(enA, 0); // Send PWM signal to L298N Enable pin
  analogWrite(enB, 0);
  delay(2000);

  //TURN
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, 100); // Send PWM signal to L298N Enable pin
  analogWrite(enB, 200);
  delay(2000);

  //STOP
  analogWrite(enA, 0); // Send PWM signal to L298N Enable pin
  analogWrite(enB, 0);
  delay(2000);

}

void loop() {
  // put your main code here, to run repeatedly:
}
