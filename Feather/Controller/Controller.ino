#define DEBUG

/**********************************************************************
   NAME: Controller.hpp
   AUTHOR: Johnathan Bizzano
   DATE: 4/20/2023
*********************************************************************/
#include <Adafruit_ISM330DHCX.h>
#include "Communication.hpp"
#include "Sensor.hpp"
#include "Motor.hpp"
#include "System.hpp"

void setup(void) {
  Serial.begin(115200);
  
  init_motors();
  init_communication();
  init_sensors();
  init_system();

  printdeviceln("Initialization Complete!");
}

void loop() { 
  update_sensors();
  update_system();

  Yield();
}
