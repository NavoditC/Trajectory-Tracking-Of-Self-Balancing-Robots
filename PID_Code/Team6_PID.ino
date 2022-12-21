/* Include necessary header files */
//#include <PinChangeInt.h>
#include <Arduino.h>
#include <MsTimer2.h>
#include <SoftwareSerial.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "KalmanFilter.h"
//#include "PinChangeInt.h"
#include "Pins.h"
#include "Wire.h"
//#include "Ultrasonic.h"

/* TB6612FNG motor driver signal pins */
#define IN1_L 7
#define IN1_R 12
#define PWM_L 5
#define PWM_R 6
#define STBY 8

//UNLTRASONIC
#define ECHO_PIN 17
#define TRIG_PIN 11

SoftwareSerial BTserial(10, 13); // RX | TX
// Connect the HC-06 TX to the Arduino RX on pin 10. 
// Connect the HC-06 RX to the Arduino TX on pin 13.

long duration = 0;
double distance = 0;
double distance_avg = 0;

/* Encoder count signal pins */
#define PinA_left 2
#define PinA_right 4

/* Sensor Reading Variables */
MPU6050 mpu; //Instantiate a MPU6050 object with the object name MPU.
KalmanFilter kalmanfilter;
int16_t ax, ay, az, gx, gy, gz; // Variables for IMU readings
volatile long right_encoder = 0;
volatile long left_encoder = 0;
/* 
 The volatile long type is used to ensure that the value is valid when the external 
 interrupt pulse count value is used in other functions 
 */
float num_of_encoder_counts_per_rev = 780.0;
float thousand_by_num_of_encoder_counts_per_rev = 1000.0/num_of_encoder_counts_per_rev;

/* IMU Callibration Variables */ 
long gyroXCalli = -218, gyroYCalli = 204, gyroZCalli = 367; // Obtained by callibrating gyroscope values
long accelXCalli = 400, accelYCalli = 8153, accelZCalli = 14840; // Obtained by callibrating accelerometer values

/* Motor PWM Input Values */
long motor_left = 0;
long motor_right = 0;

/* Time variables */
unsigned long time;
unsigned long prev_time_encoder = 0;
unsigned long startTime_left = 0;
unsigned long startTime_right = 0;
int sampling_rate = 5; // in milliseconds

/* Encoder to speed measurement variables */

int encoder_count_max = 20;
float motor_left_ang_vel = 0;
float motor_right_ang_vel = 0;
float data_MATLAB=0;
float motor_voltage=0;
/* Robot parameters */

float wheel_rad = 0.0335; // radius of the wheel in metres
float lat_dist = 0.194; // distance between ends of the wheels in metres

/* Controller parameters */

double kp_balance = 68, kd_balance = 0.78;
double kp_speed = 10, ki_speed = 0.26;
double kp_turn = 2.5, kd_turn = 0.5;
double angle_zero = 0.13;            //x axle angle calibration
double angular_velocity_zero = 1; //x axle angular velocity calibration
float dt = 0.005, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;

int encoder_left_pulse_num_speed = 0;
int encoder_right_pulse_num_speed = 0;
double speed_control_output = 0;
double rotation_control_output = 0;
double speed_filter = 0;
int speed_control_period_count = 0;
double car_speed_integeral = 0;
double speed_filter_old = 0;
int setting_car_speed = 0;
int setting_turn_speed = 0;
double pwm_left = 0;
double pwm_right = 0;

float kalmanfilter_angle;
/********************Initialization settings********************/
void setup() {
  //Ultrasonic setup
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  /* TB6612FNGN Motor Driver module control signal initialization */
  pinMode(IN1_L, OUTPUT); //Control the direction of motor 1, 1 is forward, 0 is reverse
  pinMode(IN1_R, OUTPUT); //Control the direction of motor 1, 1 is forward, 0 is reverse
  pinMode(PWM_L, OUTPUT); //PWM of left motor
  pinMode(PWM_R, OUTPUT); //PWM of right motor
  pinMode(STBY, OUTPUT); //enable TB6612FNG
  
  /* Initializing motor drive module */
  digitalWrite(IN1_L, HIGH);
  digitalWrite(IN1_R, LOW);
  digitalWrite(STBY, HIGH);
  analogWrite(PWM_L, 0);
  analogWrite(PWM_R, 0);
  
  /* Initialize I2C bus */
  Wire.begin(); //Add I2C bus sequence
  Serial.begin(57600); //Open the serial port and set the baud rate
  delay(1500);
  mpu.initialize(); //Initialization MPU6050
  delay(1500);
  //ultrasonicInit();
  delay(2);
  /* 
   Uncomment the next two lines only once and store the callibration values in the 
   global variables defined for callibration
  */
  //callibrateGyroValues();
  //callibrateAccelValues();
  
  time = millis();
  startTime_left = millis();
  startTime_right = millis();
  
  /* Interrupt function to count the encoder pulses */
  attachInterrupt(digitalPinToInterrupt(PinA_left), encoder_left, CHANGE);
  //attachPinChangeInterrupt(PinA_right, encoder_right, CHANGE);
  
  /* 
  Timing interrupt settings, using MsTimer2. Because PWM uses a timer to control the 
  duty cycle, it is important to look at the pin port corresponding to the timer when 
  using timer.
  */
  MsTimer2::set(sampling_rate, mainfunc);
  MsTimer2::start();

  /* Initialize Ultrasonic Sensor */
  Serial.print("Setup Done!\n");

  BTserial.begin(9600);
}



/***************************************************************************************/
/*
 This section contains functions that you can use to build your controllers
*/
/***************************************************************************************/

/* 
encoder_left() counts encoder pulses of the left wheel motor and stores it in a 
global variable 'left_encoder'
*/
void encoder_left(){left_encoder++;}
  
/* 
encoder_right() counts encoder pulses of the right wheel motor and stores it in a 
global variable 'right_encoder'
*/
void encoder_right(){right_encoder++;}


/* 
SetLeftWheelSpeed() takes one input which will be set as the PWM input to the left motor.
If the value is outside the range (-255,255), then the input will be saturated and the 
motor PWM will be set. The value is also written to the global variable 'motor_left'
*/
void SetLeftWheelSpeed(double speed_val)
{
  // Saturate the input to the range (-255,255)
  if (speed_val > 255) { speed_val = 255; }
  else if (speed_val < -255) { speed_val = -255; }
  
  motor_left = speed_val;
  
  if (speed_val < 0)
  {
    digitalWrite(IN1_L, 1);
    analogWrite(PWM_L, -speed_val);
  }
  else
  {
    digitalWrite(IN1_L, 0);
    analogWrite(PWM_L, speed_val);
  }
}

/* 
SetRightWheelSpeed() takes one input which will be set as the PWM input to the right motor.
If the value is outside the range (-255,255), then the input will be saturated and the 
motor PWM will be set. The value is also written to the global variable 'motor_right'
*/
void SetRightWheelSpeed(double speed_val)
{
  // Saturate the input to the range (-255,255)
  if (speed_val > 255) { speed_val = 255; }
  else if (speed_val < -255) { speed_val = -255; }
  
  motor_right = speed_val;
  
  if (speed_val < 0)
  {
    digitalWrite(IN1_R, 1);
    analogWrite(PWM_R, -speed_val);
  }
  else
  {
    digitalWrite(IN1_R, 0);
    analogWrite(PWM_R, speed_val);
  }
}

/*
readIMU() creates an MPU6050 class object and calls the function to read the six axis IMU.
The values are stored in the global variables ax,ay,az,gx,gy,gz where ax,ay,az are the 
accelerometer readings and gx,gy,gz are the gyroscope readings. 
*/
void readIMU()
{
  MPU6050 mpu_obj;
  mpu_obj.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
}

/*
readEncoder() takes the encoder pulse counts and calculates the angular velocities of the
wheels and stores it in the global variables 'motor_left_ang_vel' and 'motor_right_ang_vel'
*/
void readEncoder()
{ 
  // Encoder Calculations
  // angular velocity = (encoder_reading/num_of_counts_per_rotation)*(2*pi/sampling_time)
  
  motor_left_ang_vel = (float) 2 * 3.1415 * left_encoder * (float)thousand_by_num_of_encoder_counts_per_rev / (float)(time - startTime_left);  
  if (motor_left < 0){
    motor_left_ang_vel = -motor_left_ang_vel;}
  startTime_left = time;
  left_encoder = 0;  
  
  motor_right_ang_vel = (float) 2 * 3.1415 * right_encoder * (float)thousand_by_num_of_encoder_counts_per_rev / (float)(time - startTime_right);  
  if (motor_right < 0){
    motor_right_ang_vel = -motor_right_ang_vel;}
  startTime_right = time;
  right_encoder = 0;  
}

/* 
printIMU() prints the IMU readings to the serial monitor in the following format:
ax,ay,az,gx,gy,gz
*/
void printIMU()
{
  Serial.print("ax:");
  Serial.print(ax);
  Serial.print(',');
  Serial.print("ay:");
  Serial.print(ay);
  Serial.print(',');
  Serial.print("az:");
  Serial.print(az);
  Serial.print(',');
  Serial.print("gx:");
  Serial.print(gx);
  Serial.print(',');
  Serial.print("gy:");
  Serial.print(gy);
  Serial.print(',');
  Serial.print("gz:");
  Serial.print(gz);
  Serial.print('\n');
}

/* 
printEncoder() prints the encoder readings to the serial monitor in the following format:
motor_left_ang_vel, motor_right_ang_vel
*/
void printEncoder()
{
  Serial.print(motor_left_ang_vel);
  Serial.print(',');
  Serial.print(motor_right_ang_vel);
  Serial.print('\n');
}

/* 
printAllData() prints the IMU readings, encoder readings and PWM inputs to the motor to
the serial monitor in the following format:
ax,ay,az,gx,gy,gz,motor_left_ang_vel,motor_right_ang_vel,motor_left,motor_right
*/
void printAllData()
{
  Serial.print(time);
  Serial.print(',');
  Serial.print(ax);
  Serial.print(',');
  Serial.print(ay);
  Serial.print(',');
  Serial.print(az);
  Serial.print(',');
  Serial.print(gx);
  Serial.print(',');
  Serial.print(gy);
  Serial.print(',');
  Serial.print(gz);
  Serial.print(',');
  Serial.print(motor_left_ang_vel);
  Serial.print(',');
  Serial.print(motor_right_ang_vel);
  Serial.print(',');
  Serial.print(motor_left);
  Serial.print(',');
  Serial.print(motor_right);
  Serial.print('\n');
}

/*
callibrateGyroValues() gets the gyroscope readings n times and calculates the average of
the values to find the sensor bias. The callibration values are printed as:
gyroXCalli,gyroYCalli,gyroZCalli
*/
void callibrateGyroValues() 
{
    int n = 10000;
    for (int i=0; i < n; i++) 
    {
      readIMU();
      gyroXCalli = gyroXCalli + gx;
      gyroYCalli = gyroYCalli + gy;
      gyroZCalli = gyroZCalli + gz;
    }
    gyroXCalli = gyroXCalli/n;
    gyroYCalli = gyroYCalli/n;
    gyroZCalli = gyroZCalli/n;
    Serial.print(gyroXCalli);
    Serial.print(',');
    Serial.print(gyroYCalli);
    Serial.print(',');
    Serial.print(gyroZCalli);
    Serial.print('\n');
}

/*
callibrateAccelValues() gets the accelerometer readings n times and calculates the average of
the values to find the sensor bias. The callibration values are printed as:
accelXCalli,accelYCalli,accelZCalli
*/
void callibrateAccelValues() 
{
    int n = 10000;
    for (int i=0; i < n; i++) 
    {
      readIMU();
      accelXCalli = accelXCalli + ax;
      accelYCalli = accelYCalli + ay;
      accelZCalli = accelZCalli + az;
    }
    accelXCalli = accelXCalli/n;
    accelYCalli = accelYCalli/n;
    accelZCalli = accelZCalli/n;
    Serial.print(accelXCalli);
    Serial.print(',');
    Serial.print(accelYCalli);
    Serial.print(',');
    Serial.print(accelZCalli);
    Serial.print('\n');
}

/***************************************************************************************/
/***************** Write your custom variables and functions below *********************/
/***************************************************************************************/
const byte buffSize = 32;
char inputSeveral[buffSize]; // space for 31 chars and a terminator
byte maxChars = 12; // a shorter limit to make it easier to see what happens
                           //   if too many chars are entered
float getdatafromMATLAB()
{
    // Function credits: https://forum.arduino.cc/index.php?topic=236162.0
    // this function takes the characters from the serial input and converts them
    // to a single floating point value using the function “atof()”
    // a similar approach can be used to read an integer value if “atoi()” is used
    // first read severalChars into the array inputSeveral
    inputSeveral[0] = 0;
    byte charCount = 0;
    byte ndx = 0;
    if (BTserial.available() > 0) {
      long time = micros();
      while (BTserial.available() > 0) {
        if (ndx > maxChars - 1) {
          ndx = maxChars;
        }
        inputSeveral[ndx] = BTserial.read();
        ndx ++;
        charCount ++;
      }
      if (ndx > maxChars) {
        ndx = maxChars;
      }
      inputSeveral[ndx] = 0;
       // and then convert the string into a floating point number
      data_MATLAB = atof(inputSeveral); // atof gives 0.0 if the characters are not a valid number
      //Serial.print(“Data from MATLAB -- “);
      //Serial.println(data_MATLAB, 3); // the number specifies how many decimal places
    }
}

/*
mainfunc() is the function that is called at your specified sampling rate. The default 
sampling rate is 5ms. This function will be called at every sampling instance.
*/
void mainfunc()
{
  /* Do not modify begins*/
  sei();
  time = millis(); 
  if (time - prev_time_encoder >= encoder_count_max)
  {
    readEncoder();
    
    prev_time_encoder = time;
  }
  readIMU();
  /* Do not modify ends*/

  /* Measure Distance */
  //getDistance();
  //Serial.print("US distance: ");
  //Serial.println(distance_value);

  /*Write your code below*/
  //printEncoder();
  encoder_left_pulse_num_speed += pwm_left < 0 ? -left_encoder : left_encoder;
  encoder_right_pulse_num_speed += pwm_right < 0 ? -left_encoder : left_encoder;
  left_encoder = 0;
  right_encoder = 0;

  kalmanfilter.Angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
  kalmanfilter_angle = kalmanfilter.angle;
  double balance_control_output = kp_balance * (kalmanfilter_angle - angle_zero) + kd_balance * (kalmanfilter.Gyro_x - angular_velocity_zero);


  speed_control_period_count++;
  if (speed_control_period_count >= 8)
  {
    speed_control_period_count = 0;
    double car_speed = (encoder_left_pulse_num_speed + encoder_right_pulse_num_speed) * 0.5;
    encoder_left_pulse_num_speed = 0;
    encoder_right_pulse_num_speed = 0;
    speed_filter = speed_filter_old * 0.7 + car_speed * 0.3;
    speed_filter_old = speed_filter;
    car_speed_integeral += speed_filter;
    car_speed_integeral += -setting_car_speed;
    car_speed_integeral = constrain(car_speed_integeral, -3000, 3000);
    speed_control_output = -kp_speed * speed_filter - ki_speed * car_speed_integeral;
    rotation_control_output = setting_turn_speed + kd_turn * kalmanfilter.Gyro_z;
  }

  pwm_left = balance_control_output - speed_control_output - rotation_control_output;
  pwm_right = balance_control_output - speed_control_output + rotation_control_output;
  

  pwm_left = constrain(pwm_left, -255, 255);
  pwm_right = constrain(pwm_right, -255, 255);
  SetRightWheelSpeed(pwm_right);
  SetLeftWheelSpeed(pwm_left);  
  // SetRightWheelSpeed(0);
  // SetLeftWheelSpeed(0);  
  /***********************/
}

void loop()
{
  //ULTRASONIC BASIC TEST
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)

  //distance_avg = (distance + distance_avg)/2;
  distance_avg = distance;s
  if (distance_avg >= 20){
    setting_car_speed = 15;
  }
  else if (distance_avg <= 15){
    setting_car_speed = -15;
  }
  else{
    setting_car_speed = 0;
  }
  // BTserial.print('* ');
  // BTserial.print(distance); // Distance from Ultrasonic sensor
  // BTserial.print(' *');
  // BTserial.print('\n');
  // getdatafromMATLAB();
  // motor_voltage = data_MATLAB;
}
