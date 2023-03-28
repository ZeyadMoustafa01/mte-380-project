#include "ICM_20948.h"
#include <Wire.h>
#include <SoftwareSerial.h> // remove the inverted commas after you copy the code to the IDE
SoftwareSerial BT(10, 11); 

#define enA 5 //Brown LEFT
#define enB 6 //White RIGHT
#define in3 4 //Green RIGHT
#define in4 7 //Purple RIGHT
#define in1 3 //Red LEFT
#define in2 2 //Ornage LEFT
#define button 4
#define lSpeed 250//250
#define rSpeed 215//240
#define lSpeedBack 250
#define rSpeedBack 250
#define lSpeedTurn 80
#define rSpeedTurn 80

// Definitions for IMU
#define SERIAL_PORT Serial

#define SPI_PORT SPI     // Your desired SPI port.       Used only when "USE_SPI" is defined
#define SPI_FREQ 5000000 // You can override the default SPI frequency
#define CS_PIN 2         // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object


const int trigPin = 12;
const int echoPin = 13;
int us1Dist;
long duration;
int distance;
int count = 0;

// Variables from driving_straight.ino
//control pins for left and right motors
const int leftSpeed = enA; //means pin 9 on the Arduino controls the speed of left motor
const int rightSpeed = enB;
const int left1 = in1; //left 1 and left 2 control the direction of rotation of left motor
const int left2 = in2;
const int right1 = in3;
const int right2 = in4;

const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ; //linear acceleration
float GyroX, GyroY, GyroZ; //angular velocity
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ; //used in void loop()
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

const int maxSpeed = 250; //max PWM value written to motor speed pin. It is typically 255.
const int minSpeed = 160; //min PWM value at which motor moves
float angle; //due to how I orientated my MPU6050 on my car, angle = roll
float targetAngle = 0;
int equilibriumSpeed = 248; //rough estimate of PWM at the speed pin of the stronger motor, while driving straight 
//and weaker motor at maxSpeed
int leftSpeedVal;
int rightSpeedVal;
bool isDriving = false; //it the car driving forward OR rotate/stationary
bool prevIsDriving = true; //equals isDriving in the previous iteration of void loop()
bool paused = false; //is the program paused


void setup() {
  // put your setup code here, to run once:
  BT.begin(9600);
  BT.println("Hello from Arduino");

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
  
  bool initialized = false;
  while (!initialized)
  {
    myICM.begin(WIRE_PORT, AD0_VAL);

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
  
  // Here we are doing a SW reset to make sure the device starts in a known state
  myICM.swReset();
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    SERIAL_PORT.print(F("Software Reset returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }
  delay(250);
  // Now wake the sensor up
  myICM.sleep(false);
  myICM.lowPower(false);

  // The next few configuration functions accept a bit-mask of sensors for which the settings should be applied.

  // Set Gyro and Accelerometer to a particular sample mode
  // options: ICM_20948_Sample_Mode_Continuous
  //          ICM_20948_Sample_Mode_Cycled
  myICM.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    SERIAL_PORT.print(F("setSampleMode returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }
  
  // Set full scale ranges for both acc and gyr
  ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors

  myFSS.a = gpm2; // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                  // gpm2
                  // gpm4
                  // gpm8
                  // gpm16

  myFSS.g = dps250; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                    // dps250
                    // dps500
                    // dps1000
                    // dps2000

  myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    SERIAL_PORT.print(F("setFullScale returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }

  // Set up Digital Low-Pass Filter configuration
  ICM_20948_dlpcfg_t myDLPcfg;    // Similar to FSS, this uses a configuration structure for the desired sensors
  myDLPcfg.a = acc_d473bw_n499bw; // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
                                  // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
                                  // acc_d111bw4_n136bw
                                  // acc_d50bw4_n68bw8
                                  // acc_d23bw9_n34bw4
                                  // acc_d11bw5_n17bw
                                  // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
                                  // acc_d473bw_n499bw

  myDLPcfg.g = gyr_d361bw4_n376bw5; // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
                                    // gyr_d196bw6_n229bw8
                                    // gyr_d151bw8_n187bw6
                                    // gyr_d119bw5_n154bw3
                                    // gyr_d51bw2_n73bw3
                                    // gyr_d23bw9_n35bw9
                                    // gyr_d11bw6_n17bw8
                                    // gyr_d5bw7_n8bw9
                                    // gyr_d361bw4_n376bw5

  myICM.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    SERIAL_PORT.print(F("setDLPcfg returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }

  // Choose whether or not to use DLPF
  // Here we're also showing another way to access the status values, and that it is OK to supply individual sensor masks to these functions
  ICM_20948_Status_e accDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Acc, false);
  ICM_20948_Status_e gyrDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Gyr, false);
  SERIAL_PORT.print(F("Enable DLPF for Accelerometer returned: "));
  SERIAL_PORT.println(myICM.statusString(accDLPEnableStat));
  SERIAL_PORT.print(F("Enable DLPF for Gyroscope returned: "));
  SERIAL_PORT.println(myICM.statusString(gyrDLPEnableStat));

  // Choose whether or not to start the magnetometer
  myICM.startupMagnetometer();
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    SERIAL_PORT.print(F("startupMagnetometer returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }

  SERIAL_PORT.println();
  SERIAL_PORT.println(F("Configuration complete!"));
  
  
  // Setup frmo Driving_striaght.ino
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  // Call this function if you need to get the IMU error values for your module
  calculateError();
  delay(20);
  // IMU Setup
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  currentTime = micros();
  
  // PREVIOUS CODE
  // // GO Forward for 10 seconds
  // digitalWrite(in1, HIGH);
  // digitalWrite(in2, LOW);
  // digitalWrite(in3, HIGH);
  // digitalWrite(in4, LOW);
  // // analogWrite(enA, 250); // Send PWM signal to L298N Enable pin
  // // analogWrite(enB, 250);
  // // delay(100);

  // analogWrite(enA, lSpeed); // Send PWM signal to L298N Enable pin
  // analogWrite(enB, rSpeed);
  // delay(15000); // 10 seconds = 10 000 ms

  // SonarSensor(trigPin, echoPin);              // look bellow to find the difinition of the SonarSensor function
  // us1Dist = distance;      

  // // while(count < 1) {
  // //   SonarSensor(trigPin, echoPin);              // look bellow to find the difinition of the SonarSensor function
  // //   us1Dist = distance;  
  // //   if (us1Dist < 30) {
  // //     count += 1;
  // //   }
  // //   else {
  // //     count = 0;
  // //   }
  // // }

  // // Stop
  // analogWrite(enA, 0); // Send PWM signal to L298N Enable pin
  // analogWrite(enB, 0);
  // delay(1000);

}


void loop() {

  static int count;
  static int countStraight;
  bool isDriving = true;
  bool paused = false;
  bool prevIsDriving = false;

  if (count < 6){  
    count ++;
  } else { //runs once after void loop() runs 7 times. void loop runs about every 2.8ms, so this else condition runs every 19.6ms or 50 times/second
    count = 0;
    if (!paused){
      if (isDriving != prevIsDriving){
          leftSpeedVal = equilibriumSpeed;
          countStraight = 0;
          Serial.print("mode changed, isDriving: ");
          Serial.println(isDriving);
      }
      if (isDriving) {
        if (abs(targetAngle - angle) < 3){
          if (countStraight < 20){
            countStraight ++;
          } else {
            countStraight = 0;
            equilibriumSpeed = leftSpeedVal; //to find equilibrium speed, 20 consecutive readings need to indicate car is going straight
            Serial.print("EQUILIBRIUM reached, equilibriumSpeed: ");
            Serial.println(equilibriumSpeed);
          }
        } else {
          countStraight = 0;
        }
        driving();
      } else {
        rotate();
      }
      prevIsDriving = isDriving;
    }
  }
       
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

/**
 *  Driving Functions from driving_straight
 */

void readAcceleration() {
// Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the MPU6050 datasheet

  if (myICM.dataReady())
  {
    myICM.getAGMT();      
    AccX = myICM.accX();
    AccY = myICM.accY();
    AccZ = myICM.accZ();     // The values are only updated when you call 'getAGMT'    // This function takes into account the scale settings from when the measurement was made to calculate the values with units
  }
}

void readGyro() {
  if (myICM.dataReady())
  {
    myICM.getAGMT();      
    GyroY = myICM.gyrY();
    GyroZ = myICM.gyrZ();
    GyroX = myICM.gyrX();    // The values are only updated when you call 'getAGMT'    // This function takes into account the scale settings from when the measurement was made to calculate the values with units
  }
  
}

void driving (){//called by void loop(), which isDriving = true
  int deltaAngle = round(targetAngle - angle); //rounding is neccessary, since you never get exact values in reality
  forward();
  if (deltaAngle != 0){
    controlSpeed ();
    rightSpeedVal = maxSpeed;
    analogWrite(rightSpeed, rightSpeedVal);
    analogWrite(leftSpeed, leftSpeedVal);
  }
}

void controlSpeed (){//this function is called by driving ()
  int deltaAngle = round(targetAngle - angle);
  int targetGyroX;
  
  //setting up propoertional control, see Step 3 on the website
  if (deltaAngle > 30){
      targetGyroX = 60;
  } else if (deltaAngle < -30){
    targetGyroX = -60;
  } else {
    targetGyroX = 2 * deltaAngle;
  }
  
  if (round(targetGyroX - GyroX) == 0){
    ;
  } else if (targetGyroX > GyroX){
    leftSpeedVal = changeSpeed(leftSpeedVal, -1); //would increase GyroX
  } else {
    leftSpeedVal = changeSpeed(leftSpeedVal, +1);
  }
}

void rotate (){//called by void loop(), which isDriving = false
  int deltaAngle = round(targetAngle - angle);
  int targetGyroX;
  if (abs(deltaAngle) <= 1){
    stopCar();
  } else {
    if (angle > targetAngle) { //turn left
      left();
    } else if (angle < targetAngle) {//turn right
      right();
    }

    //setting up propoertional control, see Step 3 on the website
    if (abs(deltaAngle) > 30){
      targetGyroX = 60;
    } else {
      targetGyroX = 2 * abs(deltaAngle);
    }
    
    if (round(targetGyroX - abs(GyroX)) == 0){
      ;
    } else if (targetGyroX > abs(GyroX)){
      leftSpeedVal = changeSpeed(leftSpeedVal, +1); //would increase abs(GyroX)
    } else {
      leftSpeedVal = changeSpeed(leftSpeedVal, -1);
    }
    rightSpeedVal = leftSpeedVal;
    analogWrite(rightSpeed, rightSpeedVal);
    analogWrite(leftSpeed, leftSpeedVal);
  }
}   

int changeSpeed (int motorSpeed, int increment){
  motorSpeed += increment;
  if (motorSpeed > maxSpeed){ //to prevent motorSpeed from exceeding 255, which is a problem when using analogWrite
    motorSpeed = maxSpeed;
  } else if (motorSpeed < minSpeed){
    motorSpeed = minSpeed;
  }
  return motorSpeed;
}

void calculateError() {
  //When this function is called, ensure the car is stationary. See Step 2 for more info
  
  // Read accelerometer values 200 times
  c = 0;
  while (c < 200) {
    readAcceleration();
    // Sum all readings
    AccErrorX += (atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI);
    AccErrorY += (atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI);
    c++;
  }
  //Divide the sum by 200 to get the error value, since expected value of reading is zero
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  
  // Read gyro values 200 times
  while (c < 200) {
    readGyro();
    // Sum all readings
    GyroErrorX += GyroX;
    GyroErrorY += GyroY;
    GyroErrorZ += GyroZ;
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  Serial.println("The the gryoscope setting in MPU6050 has been calibrated");
}

void stopCar(){
  digitalWrite(right1, LOW);
  digitalWrite(right2, LOW);
  digitalWrite(left1, LOW);
  digitalWrite(left2, LOW);
  analogWrite(rightSpeed, 0);
  analogWrite(leftSpeed, 0);
}

void forward(){ //drives the car forward, assuming leftSpeedVal and rightSpeedVal are set high enough
  digitalWrite(right1, HIGH); //the right motor rotates FORWARDS when right1 is HIGH and right2 is LOW
  digitalWrite(right2, LOW);
  digitalWrite(left1, HIGH);
  digitalWrite(left2, LOW);
}

void left(){ //rotates the car left, assuming speed leftSpeedVal and rightSpeedVal are set high enough
  digitalWrite(right1, LOW);
  digitalWrite(right2, HIGH);
  digitalWrite(left1, HIGH);
  digitalWrite(left2, LOW);
}

void right(){
  digitalWrite(right1, HIGH);
  digitalWrite(right2, LOW);
  digitalWrite(left1, LOW);
  digitalWrite(left2, HIGH);
}

