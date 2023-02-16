const int trigPin = 12;
const int echoPin = 13;
const int trig2Pin = 11;
const int echo2Pin = 10;
const int trig3Pin = 9;
const int echo3Pin = 8;

long duration;
int distance;
long duration2;
int distance2;
long duration3;
int distance3;

long UltraSensor1, UltraSensor2, UltraSensor3;

void setup() {
  // put your setup code here, to run once:
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(trig2Pin, OUTPUT);
  pinMode(echo2Pin, INPUT);
  pinMode(trig3Pin, OUTPUT);
  pinMode(echo3Pin, INPUT);
  Serial.begin(9600);
}

void loop() {
  SonarSensor(trigPin, echoPin);              // look bellow to find the difinition of the SonarSensor function
  UltraSensor1 = distance;                      // store the distance in the first variable
  SonarSensor(trig2Pin,echo2Pin);               // call the SonarSensor function again with the second sensor pins
  UltraSensor2 = distance;    
  SonarSensor(trig3Pin, echo3Pin);
  UltraSensor3 = distance;

  Serial.print("distance measured: ");
  Serial.print(UltraSensor1);
  Serial.print(" cm ");
  Serial.print(UltraSensor2);
  Serial.print(" cm ");
  Serial.print(UltraSensor3);
  Serial.println(" cm");
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