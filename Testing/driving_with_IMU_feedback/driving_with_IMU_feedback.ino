#include "ICM_20948.h"

#define SERIAL_PORT Serial

#define SPI_PORT SPI     // Your desired SPI port.       Used only when "USE_SPI" is defined
#define SPI_FREQ 5000000 // You can override the default SPI frequency
#define CS_PIN 2         // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

#include <SoftwareSerial.h>
SoftwareSerial BT(10, 11);


ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object


#include <Wire.h>
//control pins for left and right motors
const int leftSpeed = 5; //enA //means pin 9 on the Arduino controls the speed of left motor
const int rightSpeed = 6; //enB
const int left1 = 4; //in3 //left 1 and left 2 control the direction of rotation of left motor
const int left2 = 7; //in4
const int right1 = 3; //in1
const int right2 = 2; //in2

int left_rotation = 230;
int right_rotation = 230;


float AccX, AccY, AccZ; //linear acceleration
float GyroX, GyroY, GyroZ; //angular velocity
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ; //used in void loop()
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
float start_time, drive_duration, end_time;
float sampling_start, sampling_duration;
int c = 0;

const int trigPin = 12;
const int echoPin = 13;
int us1Dist;
long duration;
int distance;

void setup() {
  delay(5000);
  setup_IMU();
  BT.begin(9600);
  delay(20);
  pinMode(left1, OUTPUT);
  pinMode(left2, OUTPUT);
  pinMode(right1, OUTPUT);
  pinMode(right2, OUTPUT);
  pinMode(leftSpeed, OUTPUT);
  pinMode(rightSpeed, OUTPUT);
  
  //Ultrasonic
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  calculateError();

  demo_run();
  

}

void loop() {
  // put your main code here, to run repeatedly:

}

void setup_IMU() {

  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT)
  {
  };
  #ifdef USE_SPI
  SPI_PORT.begin(115200);
  #else
    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);
  #endif

  bool initialized = false;
  while (!initialized)
  {

  #ifdef USE_SPI
      myICM.begin(CS_PIN, SPI_PORT, SPI_FREQ); // Here we are using the user-defined SPI_FREQ as the clock speed of the SPI bus
  #else
      myICM.begin(WIRE_PORT, AD0_VAL);
  #endif

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
}

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

void readGyro() {
  if (myICM.dataReady())
  {
    myICM.getAGMT();      
    GyroY = myICM.gyrY();
    GyroZ = myICM.gyrZ();
    GyroX = myICM.gyrX();    // The values are only updated when you call 'getAGMT'    // This function takes into account the scale settings from when the measurement was made to calculate the values with units
  }
  
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

void drive_straight_distance(float drive_distance) {
  set_forward();
  update_timing();

  analogWrite(leftSpeed, left_rotation);
  analogWrite(rightSpeed, right_rotation);
  update_us_1();

  while (us1Dist > drive_distance) {
    update_z_angle();
    print_sample('Z', gyroAngleZ);
    update_speeds();
    update_us_1();
  }
  stop_robot();

}

void drive_straight_time(float drive_duration) {
  set_forward();
  float end_time;
  end_time = currentTime + drive_duration*1000;
  update_timing();

  while (end_time > currentTime) {
    update_z_angle();
    print_sample('Z', gyroAngleZ);

    update_speeds();
  }
  stop_robot();

}

void turn_90() {
  turnleft();
  update_timing();

  while (gyroAngleZ < 90) {
    update_z_angle();
    print_sample('Z', gyroAngleZ);

    analogWrite(leftSpeed, 80);
    analogWrite(rightSpeed, 80);
  }
  stop_robot();
  reset_angles();
}

void reset_angles() {
  gyroAngleZ = 0;
  gyroAngleX = 0;
  gyroAngleY = 0;
}

void find_ramp() {
  drive_straight_distance(5);
}

void going_up_ramp() {

  currentTime = micros();
  sampling_start = micros();

  while (gyroAngleX < 20) {
    drive_straight();
    update_x_angle();
  }
  while (gyroAngleX > 20) {
  }
}

void demo_run() {
  drive_straight_distance(15);
  turn_90();
  drive_straight_time(2000);
}

void drive_straight() {
  set_forward();

  currentTime = micros();
  sampling_start = micros();

  analogWrite(leftSpeed, left_rotation);
  analogWrite(rightSpeed, right_rotation);

  update_z_angle();
  print_sample("Z", gyroAngleZ);
  update_speeds();

}

void stop_robot() {
  analogWrite(leftSpeed, 0);
  analogWrite(rightSpeed, 0);
}

void set_forward() {
  digitalWrite(right1, HIGH);
  digitalWrite(right2, LOW);
  digitalWrite(left1, HIGH);
  digitalWrite(left2, LOW);
}

void turnleft() {
  digitalWrite(right1, LOW);
  digitalWrite(right2, HIGH);
  digitalWrite(left1, HIGH);
  digitalWrite(left2, LOW);
}

void update_z_angle() {
  readGyro();
  
  previousTime = currentTime;
  currentTime = micros();
  elapsedTime = (currentTime - previousTime) / 1000000;
  
  GyroZ -= GyroErrorZ;

  if (abs(GyroZ) > 5) {
    gyroAngleZ += GyroZ * elapsedTime;
  }
}

void print_sample(char angle_name, float value) {
  sampling_duration = (currentTime - sampling_start) / 1000;
  if (sampling_duration > 500) {
    BT.print(angle_name);
    BT.print("-angle: ");
    BT.println(value);
    sampling_start = currentTime;
  }
}

void update_speeds() {
    if (gyroAngleZ > 1) {
    right_rotation -= 5;//right_rotation = 100; //right_rotation -= 5
    // left_rotation = 230;
    analogWrite(leftSpeed, left_rotation);
    analogWrite(rightSpeed, right_rotation);
  }
  else if (gyroAngleZ < -1) {
    left_rotation -= 5;// left_rotation = 100; //right_rotation -= 5
    // right_rotation = 230;
    analogWrite(leftSpeed, left_rotation);
    analogWrite(rightSpeed, right_rotation);     
  }
  else {
    right_rotation = 230;
    left_rotation = 230;
    analogWrite(rightSpeed, right_rotation);
    analogWrite(leftSpeed, left_rotation);  
  }
}

void update_x_angle() {
  readGyro();
  
  previousTime = currentTime;
  currentTime = micros();
  elapsedTime = (currentTime - previousTime) / 1000000;
  
  GyroX -= GyroErrorX;

  if (abs(GyroX) > 5) {
    gyroAngleX += GyroX * elapsedTime;
  }
}

void update_timing() {
  currentTime = micros();
  sampling_start = micros();
}

void update_us_1() {
  SonarSensor(trigPin, echoPin);
  us1Dist = distance;  
}
