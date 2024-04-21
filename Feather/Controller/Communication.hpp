#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <SimpleConnection.hpp>
#include <SimpleTimer.hpp>
#include <devices/SimpleFeather.hpp>

#define RFM95_Slave 8
#define RFM95_Reset 4
#define RFM95_Interrupt 3
#define RF95_FREQ 915.0
#define RF95_POWER 14

#define AttitudePacketInterval 500

void radio_print(const char* fmt, ...);
#undef print
#define print(fmt, ...) radio_print(fmt, ##__VA_ARGS__)
#define printdevice(fmt, ...) print("[Device]:" fmt, ##__VA_ARGS__)
#define printdeviceln(fmt, ...) println("[Device]:" fmt, ##__VA_ARGS__)

enum PacketType : uint8_t{
  ComputerPrint = 1,
  AttitudeUpdate = 2
};

enum Device : uint8_t{
  Device = 0,
  Cntrl = 1,  
};

struct DeviceRadioConnection : public RadioConnection{
  DeviceRadioConnection() : RadioConnection(RFM95_Slave, RFM95_Interrupt, RFM95_Reset, 256) { SetAddress(Device); }

  void SendPacket(RadioPacket* p){
    p->SeekStart();
    Send(p);
  }

  void Receive(RadioPacket* io) final;
};

Timer attitudePacketTimer(true, AttitudePacketInterval);
DeviceRadioConnection device;
RadioPacket rp1 = RadioPacket(256);

//Simple::Printf implementation stream to Rx
void radio_print(const char* fmt, ...){
  va_list args;
  va_start(args, fmt);

  rp1.config(Cntrl, PacketType::ComputerPrint);
  rp1.vPrintf((char*) fmt, args);
  device.SendPacket(&rp1);

  debugOnly( Out.vPrintf((char*) fmt, args); )

  va_end(args);
}

void init_communication(){
  if(!device.Initialize(RF95_FREQ, RF95_POWER, Range::Medium)){
    Serial.printf("LoRa Radio Initialization Failed!");
    return;
  }
  printdeviceln("LoRa Okay!");
  device.Start();
}

//Method called when a packet from the Feather Connection Pool is received
void DeviceRadioConnection::Receive(RadioPacket* p) {
  switch(p->id){

  }
}

#endif