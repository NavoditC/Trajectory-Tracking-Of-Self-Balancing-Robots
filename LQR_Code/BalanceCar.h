/*
   @Description: In User Settings Edit
   @Author: your name
   @Date: 2019-10-08 09:35:07
   @LastEditTime: 2019-10-11 16:25:04
   @LastEditors: Please set LastEditors
*/
#include "MsTimer2.h"
#include "KalmanFilter.h"
#include "I2Cdev.h"
//#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
#include "Wire.h"

// Import gains and state matrices here
#include "gains.h"

#define tumbllerOffset 30

// Declare self-defined functions here
void updateCar();
void readEncoderSpeed();
void resetEncoderSpeed();
void readIMU();
void writePWMToPins();
void writePWMfromVoltage();

void getMeasurementsElegoo();
void getMeasurementsStandard();
void getFilteredMeasurements();
double lowPassFilter10(double& prevY, double& prevU, double& curU);
double lowPassFilter15(double& prevY, double& prevU, double& curU);
double lowPassFilter20(double& prevY, double& prevU, double& curU);

void updateFilteredEncoderSpeed();
void updateObservedStates();
void useDirectMeasurements();
void useFilteredMeasurements();
void getVoltageFromLQR();

void printCurrentMeasurements();
void printFilteredMeasurements();
void printCurrentStates();

// Define MPU and Kalman filter classes
MPU6050 mpu;
KalmanFilter kalmanfilter;

// Setting MPU6050 calibration parameters
double angle_zero = 0;            //x axle angle calibration
double angular_velocity_zero = 0; //x axle angular velocity calibration
int16_t ax, ay, az, gx, gy, gz;
float dt = 0.005, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;

// Setting conversion units
float counts_per_rev = 780.0;
float wheel_radius = 0.0335;
float voltage_to_pwm = 255 / 7.25; // pwm counts per volt

// Motor, encoder and state readings
volatile unsigned long encoder_count_right_a = 0;
volatile unsigned long encoder_count_left_a = 0;
int encoder_left_pulse_num_speed = 0;
int encoder_right_pulse_num_speed = 0;
int speed_control_period_count = 0;
float car_encoder_speed_filter_old = 0;

double pwm_left = 0;
double pwm_right = 0;
float balance_angle_max = 20 * PI / 180;
float balance_angle_min = -balance_angle_max;
float initial_angle_max = 10 * PI / 180;
float initial_angle_min = -initial_angle_max;
bool carActivated = false;

// Actual states of the robot
double currentStates[4] = {0.0, 0.0, 0.0, 0.0};
double prevStates[4] = {0.0, 0.0, 0.0, 0.0};

// Measurements. FilteredMeasurements requires currentMeasurements and prevMeasurements for LPF
double filteredMeasurements[4] = {0.0, 0.0, 0.0, 0.0};
double currentMeasurements[4] = {0.0, 0.0, 0.0, 0.0};
double prevMeasurements[4] = {0.0, 0.0, 0.0, 0.0};

// Input of the robot
double prevVoltage;
double voltage;

void recomputeVoltageToPWM()
{
  double max_voltage = (analogRead(VOL_MEASURE_PIN) * 1.1 / 1024) * ((10 + 1.5) / 1.5);
  voltage_to_pwm = 255 / max_voltage;
}

void carStop()
{
  // Nullify states and last pwm
  carActivated = false;
  currentMeasurements[0] = 0;
  currentMeasurements[1] = 0;
  for (int i = 0; i < 4; ++i)
  {
    currentStates[i] = currentMeasurements[i];
    prevStates[i] = currentMeasurements[i];
    prevMeasurements[i] = currentMeasurements[i];
    filteredMeasurements[i] = currentMeasurements[i];
  }
  voltage = 0;
  prevVoltage = 0;
  pwm_left = 0;
  pwm_right = 0;

  // Nullify pins
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(STBY_PIN, HIGH);
  analogWrite(PWMA_LEFT, pwm_left);
  analogWrite(PWMB_RIGHT, pwm_right);
}

void balanceCar()
{
  // Call interrupt
  sei();

  // Accumulate encoder counts for past 5ms and reset
  readEncoderSpeed();

  // Get MPU data, pendulum angle, and ultrasonic distance at every 5ms
  readIMU();
  getDistance();

  
  // Get measurements from encoder and IMU
  //   getMeasurementsElegoo(); // Elegoo method. If using this, remember to change observed state
  getMeasurementsStandard(); // Raw measurements
  getFilteredMeasurements();

  // Observe states (Prediction first)
  updateObservedStates();

  // Controller voltage
  getVoltageFromLQR();

  // Generate PWM
  writePWMfromVoltage();

  // Check angle conditions and updates car states
  updateCar();

  // Print statements. Adds to processing delay. Best to comment out if not required.
  //  printCurrentMeasurements();
    printCurrentStates();
  //  Serial.println(" ");
}

void updateCar() {
  if (carActivated) {
    if (currentStates[2] < balance_angle_min || balance_angle_max < currentStates[2])
    {
      carStop();
    }

    else
    {
      writePWMToPins();
    }
  }

  else if (currentStates[2] > initial_angle_min && initial_angle_max > currentStates[2])
  {
    carActivated = true;
    writePWMToPins();
  }

  else {
    carStop();
  }
}

void readIMU()
{
  // Note: Kalmanfilter.Angle estimates our current angle of the robot.
  // Function fuses gyro and accelerometer values via kalman and complementary to get variables -- angle and angle6 respectively
  // Choose EITHER angle/angle6 as your measurement
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  kalmanfilter.Angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1); // Calculates angle6 (complementary) and angle (kalman)
}

void readEncoderSpeed()
{
  encoder_left_pulse_num_speed += pwm_left < 0 ? -encoder_count_left_a : encoder_count_left_a;
  encoder_right_pulse_num_speed += pwm_right < 0 ? -encoder_count_right_a : encoder_count_right_a;
  encoder_count_left_a = 0;
  encoder_count_right_a = 0;
}

void resetEncoderSpeed()
{
  encoder_left_pulse_num_speed = 0;
  encoder_right_pulse_num_speed = 0;
  speed_control_period_count = 0;
}

void updateFilteredEncoderSpeed()
{
  // Compute FILTERED encoder speed
  double car_encoder_speed = (encoder_left_pulse_num_speed + encoder_right_pulse_num_speed) * 0.5; // counts since last interrupt
  car_encoder_speed = car_encoder_speed_filter_old * 0.7 + car_encoder_speed * 0.3; // counts since last
  car_encoder_speed_filter_old = car_encoder_speed;
}

void getMeasurementsElegoo() {
  // Gets measurements using the Elegoo method

  int speed_cycle_interval = 8;
  float speed_cycle_time = (float)speed_cycle_interval * dt;
  float speed_const = (2.0 * PI * wheel_radius / counts_per_rev) * (1.0 / speed_cycle_time); // meters per count * cycles per second - counts per cycle * speed_const = meters per second

  speed_control_period_count++;

  for (int i = 0; i < 4; ++i) {
    prevMeasurements[i] = currentMeasurements[i];
  }

  if (speed_control_period_count >= speed_cycle_interval) {
    updateFilteredEncoderSpeed();
    resetEncoderSpeed();
    currentMeasurements[1] = speed_const * car_encoder_speed_filter_old;
    currentMeasurements[0] += currentMeasurements[1] * speed_cycle_time;
  }

  currentMeasurements[2] = -kalmanfilter.angle * PI / 180; // Kalman
  currentMeasurements[3] = -(kalmanfilter.Gyro_x - angular_velocity_zero) * PI / 180;
}

void getMeasurementsStandard()
{
  // Note: Kalmanfilter.Angle estimates our current angle of the robot.
  // Function fuses gyro and accelerometer values via kalman and complementary to get variables -- angle and angle6 respectively
  // Choose EITHER angle/angle6 as your measurement
  float speed_const = (2.0 * PI * wheel_radius / counts_per_rev) * (1.0 / dt);

  for (int i = 0; i < 4; ++i) {
    prevMeasurements[i] = currentMeasurements[i];
  }

  
  currentMeasurements[1] = speed_const * (encoder_left_pulse_num_speed + encoder_right_pulse_num_speed) * 0.5;
  // currentMeasurements[0] += currentMeasurements[1] * dt;
  currentMeasurements[2] = -kalmanfilter.angle * PI / 180; // Kalman
  // currentMeasurements[2] = -kalmanfilter.angle6 * PI / 180; // Complementary
  currentMeasurements[0] = (tumbllerOffset - (distance_value * cos(currentMeasurements[2])))*0.01; // Correct x distance error introduced from angle of Tumbller
  currentMeasurements[3] = -(kalmanfilter.Gyro_x - angular_velocity_zero) * PI / 180;
  resetEncoderSpeed();
}

void getFilteredMeasurements()
{
  filteredMeasurements[0] = lowPassFilter20(filteredMeasurements[0], prevMeasurements[0], currentMeasurements[0]);
  filteredMeasurements[1] = lowPassFilter20(filteredMeasurements[1], prevMeasurements[1], currentMeasurements[1]);
  filteredMeasurements[2] = currentMeasurements[2]; // lowPassFilter20(filteredMeasurements[2], prevMeasurements[2], currentMeasurements[2]);
  filteredMeasurements[3] = currentMeasurements[3]; // lowPassFilter20(filteredMeasurements[3], prevMeasurements[3], currentMeasurements[3]);
}

double lowPassFilter10(double& prevY, double& prevU, double& curU)
{
  // Low pass filter 10Hz. Tustin
  double b = 0.9512; // Obtained from Matlab
  double curY = b * prevY + (1 - b) / 2 * curU + (1 - b) / 2 * prevU;
  return curY;
}

double lowPassFilter15(double& prevY, double& prevU, double& curU)
{
  // Low pass filter 15Hz. Tustin
  double b = 0.9277; // Obtained from Matlab
  double curY = b * prevY + (1 - b) / 2 * curU + (1 - b) / 2 * prevU;
  return curY;
}

double lowPassFilter20(double& prevY, double& prevU, double& curU)
{
  // Low pass filter 20Hz. Tustin
  double b = 0.9048; // Obtained from Matlab
  double curY = b * prevY + (1 - b) / 2 * curU + (1 - b) / 2 * prevU;
  return curY;
}

void updateObservedStates() {
  // Note: All measurements and current/previous states are global
  // ABCDLK matrices are also global variables
  // Here, we assume that we measure angle, angularvel, and displacement

  //    useDirectMeasurements();
  useFilteredMeasurements();
}

void useDirectMeasurements()
{
  for (int j = 0; j < 4; ++j) {
    currentStates[j] = currentMeasurements[j];
    prevStates[j] = currentStates[j];
  }
}

void useFilteredMeasurements()
{
  for (int j = 0; j < 4; ++j) {
    currentStates[j] = filteredMeasurements[j];
    prevStates[j] = currentStates[j];
  }
}

void getVoltageFromLQR() {
  voltage = 0;

  for (int i = 0; i < 4; ++i) {
    voltage -= K[i] * currentStates[i];
  }

  prevVoltage = voltage;
}

void printCurrentMeasurements()
{
  //  Serial.print("Current Measurements: ");
  for (int i = 0; i < 4; ++i)
  {
    Serial.print(currentMeasurements[i]);
    Serial.print(" ");
  }
  Serial.println(" ");
}

void printfilteredMeasurements()
{
  Serial.print("Filtered Measurements: ");
  for (int i = 0; i < 4; ++i)
  {
    Serial.print(filteredMeasurements[i]);
    Serial.print(" ");
  }
  Serial.println(" ");
}

void printCurrentStates()
{
  //  Serial.print("Current States: ");
  for (int i = 0; i < 4; ++i)
  {
    Serial.print(currentStates[i]);
    Serial.print(" ");
  }
  Serial.println(" ");
}

void writePWMfromVoltage()
{
  pwm_left = voltage * voltage_to_pwm;
  pwm_right = pwm_left;
  pwm_left = constrain(pwm_left, -255, 255);
  pwm_right = constrain(pwm_right, -255, 255);
}

void writePWMToPins()
{
  if (pwm_left < 0)
  {
    digitalWrite(AIN1, 1);
    analogWrite(PWMA_LEFT, -pwm_left);
  }
  else
  {
    digitalWrite(AIN1, 0);
    analogWrite(PWMA_LEFT, pwm_left);
  }

  if (pwm_right < 0)
  {
    digitalWrite(BIN1, 1);
    analogWrite(PWMB_RIGHT, -pwm_right);
  }
  else
  {
    digitalWrite(BIN1, 0);
    analogWrite(PWMB_RIGHT, pwm_right);
  }
}

void encoderCountRightA()
{
  encoder_count_right_a++;
}

void encoderCountLeftA()
{
  encoder_count_left_a++;
}

void carInitialize()
{
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(PWMA_LEFT, OUTPUT);
  pinMode(PWMB_RIGHT, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);
  carStop();
  Wire.begin();
  mpu.initialize();
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A_PIN), encoderCountLeftA, CHANGE);
  attachPinChangeInterrupt(ENCODER_RIGHT_A_PIN, encoderCountRightA, CHANGE);
  MsTimer2::set(5, balanceCar);
  MsTimer2::start();
}
