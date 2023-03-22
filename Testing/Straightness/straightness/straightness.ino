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
const int trigPinRight = 9;
const int echoPinRight = 8;
int us1Dist;
int usDistRight;
int usDistRight_temp;
long duration;
int distance;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

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
  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinRight, INPUT);
  
  for (int i = 0; i < 10; i++){
    SonarSensor(trigPinRight, echoPinRight);
    usDistRight += distance;
    delay(100);
  }

  usDistRight /= 10;

  // GO Forward for 10 seconds
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, lSpeed);
  analogWrite(enB, rSpeed);

  SonarSensor(trigPin, echoPin);
  us1Dist = distance;

  

  while(us1Dist > 30) {
    SonarSensor(trigPin, echoPin);
    us1Dist = distance;
    SonarSensor(trigPinRight, echoPinRight);
    usDistRight_temp = distance;
    Serial.print("Difference between original and measured: ");
    Serial.println(usDistRight - usDistRight_temp);
    Serial.print("Measured distance: ");
    Serial.println(usDistRight_temp);
    Serial.print("Orginal distance: ");
    Serial.println(usDistRight);
    delay(100);
    // if (abs(usDistRight - usDistRight_temp) > 5) {
    //   break;
    // }
  }

  analogWrite(enA, 0);
  analogWrite(enB, 0);
  delay(1000);

}

/**
 * Go straight ahead
 */
void goStraight()
{
    // GO Forward for 10 seconds
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, lSpeed); // Send PWM signal to L298N Enable pin
  analogWrite(enB, rSpeed);
}

/**
 * Adjust the speed by slowing down the wheel of the opposite direction until back to origin.
 * Assume measurements are from the right US
 * veer_r: True if the car has veered to the right, False if to the left.
 * corr: The amount correction is required.
 */
void adjustDir(bool veer_r, int corr)
{
  unsigned long curr_t = 0;;
  unsigned long after_t = 0;

  SonarSensor(trigPin, echoPin);
  int dist_corr = distance + corr * (veer_r ? -1 : 1);

  // Set direction to forwards (just in case)
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  curr_t = millis();
  // Lower the speed of one of the motors to adjust the direction.
  analogWrite(enA, lSpeed - (veer_r ? 50 : 0));
  analogWrite(enB, rSpeed - (veer_r ? 0 : 50));

  SonarSensor(trigPin, echoPin);

  while (abs(dist_corr - distance) > 5)
  {
    SonarSensor(trigPin, echoPin);
    delay(1000);
  }

  after_t = millis();


  analogWrite(enA, lSpeed - (!veer_r ? 50 : 0));
  analogWrite(enB, rSpeed - (!veer_r ? 0 : 50));

  delay(after_t - curr_t);

  goStraight();

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