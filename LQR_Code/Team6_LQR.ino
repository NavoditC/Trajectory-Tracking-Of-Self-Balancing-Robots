 /*
Tumbller LQR code using Segway dynamics
 */

#include <Arduino.h>
#include "PinChangeInt.h"
#include "Pins.h"
#include "mode.h"
#include "Ultrasonic.h"
#include "BalanceCar.h"
#include "voltage.h"

unsigned long start_prev_time = 0;


void setup()
{

  Serial.begin(115200);
  voltageInit();
  start_prev_time = millis();
  carInitialize(); // Interrupt timer to call balanceCar() func. every 5ms 
  ultrasonicInit(); // Connect Interrupt for ultrasonic.
}

void loop()
{
  voltageMeasure();
  static unsigned long print_time;
//  if (millis() - print_time > 10)
//  {
//    print_time = millis();
//  }

  // Stops car for the first 100ms
  static unsigned long start_time;
  if (millis() - start_time < 100)
  {
    carStop();
  }
}
