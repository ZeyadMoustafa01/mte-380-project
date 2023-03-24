#define enA 5 //Brown
#define enB 6 //White
#define in3 4 //Green
#define in4 7 //Purple
#define in1 3 //Red
#define in2 2 //Ornage
#define button 4
#define lSpeed 250//250
#define rSpeed 210//240
#define lSpeedBack 250
#define rSpeedBack 250
#define lSpeedTurn 80
#define rSpeedTurn 80

const int trigPin = 12;
const int echoPin = 13;
int us1Dist;
long duration;
int distance;
int count = 0;

void setup() {
  // put your setup code here, to run once:

  //Motor Setup
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  //US setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  delay(5000);
  
  // GO Forward for 10 seconds
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  // analogWrite(enA, 250); // Send PWM signal to L298N Enable pin
  // analogWrite(enB, 250);
  // delay(100);

  analogWrite(enA, lSpeed); // Send PWM signal to L298N Enable pin
  analogWrite(enB, rSpeed);
  // delay(10000); // 10 seconds = 10 000 ms

  SonarSensor(trigPin, echoPin);              // look bellow to find the difinition of the SonarSensor function
  us1Dist = distance;      

  while(count < 1) {
    SonarSensor(trigPin, echoPin);              // look bellow to find the difinition of the SonarSensor function
    us1Dist = distance;  
    if (us1Dist < 30) {
      count += 1;
    }
    else {
      count = 0;
    }
  }

  // Stop
  analogWrite(enA, 0); // Send PWM signal to L298N Enable pin
  analogWrite(enB, 0);
  delay(1000);

}


void loop() {
  // Leave empty 
}

// SonarSensor function used to generate and read the ultrasonic wave
void SonarSensor(int trigPinSensor,int echoPinSensor)//it takes the trigPIN and the echoPIN 
{
  //START SonarSensor FUNCTION
  //generate the ultrasonic wave
  //---------------------------------------------------------------------------------------------------------------------- 
  digitalWrite(trigPinSensor, LOW);// put trigpin LOW 
  delayMicroseconds(2);// wait 2 microseconds
  digitalWrite(trigPinSensor, HIGH);// switch trigpin HIGH
  delayMicroseconds(10); // wait 10 microseconds
  digitalWrite(trigPinSensor, LOW);// turn it LOW again
  //----------------------------------------------------------------------------------------------------------------------

  //read the distance
  //----------------------------------------------------------------------------------------------------------------------
  duration = pulseIn(echoPinSensor, HIGH);//pulseIn funtion will return the time on how much the configured pin remain the level HIGH or LOW; in this case it will return how much time echoPinSensor stay HIGH
  distance= duration * 0.034/2; // first we have to divide the duration by two  
}// END SonarSensor FUNCTION