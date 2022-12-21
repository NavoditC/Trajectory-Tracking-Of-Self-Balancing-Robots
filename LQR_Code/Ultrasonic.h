char measure_flag = 0;
double distance_value = 0;
double prev_distance_value = 0;
unsigned long measure_prev_time = 0;
unsigned long get_distance_prev_time = 0;

void ultrasonicInit()
{
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
}

void measureDistance()
{
  if (measure_flag == 0)
  {
    measure_prev_time = micros();
    attachPinChangeInterrupt(ECHO_PIN, measureDistance, FALLING);
    measure_flag = 1;
  }
  else if (measure_flag == 1)
  {
    prev_distance_value = distance_value;
    distance_value = (micros() - measure_prev_time) * 0.017; //340.29 m/s / 2 -> (340.29*100 cm) /(1000*1000 us) / 2 = 0.0170145
    if(distance_value > 200) distance_value = prev_distance_value;
    
    measure_flag = 2;
  }
}

void getDistance()
{
  if (millis() - get_distance_prev_time > 50)
  {
    get_distance_prev_time = millis();
    measure_flag = 0;
    attachPinChangeInterrupt(ECHO_PIN, measureDistance, RISING);
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
  }
}
