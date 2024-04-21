#ifndef SENSOR_H
#define SENSOR_H

#include "Communication.hpp"

#define MeasurementInterval 500

Adafruit_ISM330DHCX ism330dhcx;

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;

void measureGyro(){
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  ism330dhcx.getEvent(&accel, &gyro, &temp);  
}

void init_sensors(){
  if (!ism330dhcx.begin_I2C()) {
    printdeviceln("Failed to find ISM330DHCX chip");
    while (1) {
      delay(10);
    }
  }

  printdeviceln("ISM330DHCX Found!");
  ism330dhcx.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  ism330dhcx.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS); //ISM330DHCX_GYRO_RANGE_2000_DPS);
  ism330dhcx.setAccelDataRate(LSM6DS_RATE_1_66K_HZ);
  ism330dhcx.setGyroDataRate(LSM6DS_RATE_1_66K_HZ);
  ism330dhcx.configInt1(false, false, true); // accelerometer DRDY on INT1
  ism330dhcx.configInt2(false, true, false); // gyro DRDY on INT2

  attitudePacketTimer.callback = make_static_lambda(void, (Timer& t), {
    printdeviceln("\t\tAccel X: %f Y:%f Z:%f m/s^2", accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);
    printdeviceln("\t\tGyro X: %f Y:%f Z:%f radians/s", gyro.gyro.x, gyro.gyro.y, gyro.gyro.z);

   /* rp1.config(Cntrl, PacketType::AttitudeUpdate);
    rp1.WriteStd(Gz);
    device.SendPacket(&rp1);
    */

    Serial.printf("Sent Packet! \n");
  });

  attitudePacketTimer.Start();
}

void update_sensors(){
  ism330dhcx.getEvent(&accel, &gyro, &temp);
}

#endif