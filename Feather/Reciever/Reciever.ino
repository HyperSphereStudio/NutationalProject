#define DEBUG

#include <SimpleConnection.hpp>
#include <SimpleTimer.hpp>
#include <devices/SimpleFeather.hpp>

#define RFM95_Slave 8
#define RFM95_Reset 4
#define RFM95_Interrupt 3
#define RF95_FREQ 915.0
#define RF95_POWER 14

void radio_print(const char* fmt, ...);
#define print(fmt, ...) radio_print(fmt, ##__VA_ARGS__)
#define printcntrl(fmt, ...) print("[Cntrl]:" fmt, ##__VA_ARGS__)
#define printcntrlln(fmt, ...) println("[Cntrl]:" fmt, ##__VA_ARGS__)

enum PacketType : uint8_t{
  ComputerPrint = 1,
  AttitudeUpdate = 2,
};

enum Device : uint8_t{
  Device = 0,
  Cntrl = 1,  
};

struct SimpleComputerPacket : public Packet{
  uint8_t id;

  SimpleComputerPacket(int capacity) : Packet(capacity){}

  void config(uint8_t type, bool reset=true){
    id = type;
    if(reset)
      Clear();
  }
};

struct CntrlRadioConnection : public RadioConnection{
  CntrlRadioConnection() : RadioConnection(RFM95_Slave, RFM95_Interrupt, RFM95_Reset, 256) { SetAddress(Cntrl); }

  void SendPacket(RadioPacket* p){
    p->SeekStart();
    Send(p);
  }

  void Receive(RadioPacket* io) final;
};

struct CntrlCompConnection;

struct CntrlSimpleCompConnection : public SimpleConnection{
    CntrlCompConnection* msc;
    CntrlSimpleCompConnection(CntrlCompConnection* c) : msc(c){}

    void ReceivedMessage(Packet* io) final;

    protected:
    void Write(IO* io) final;
};

//Extension of a serial connection for the computer to handle the packets
struct CntrlCompConnection : public SerialConnection{
  friend CntrlSimpleCompConnection;
  
  CntrlSimpleCompConnection sc;

  CntrlCompConnection() : sc(this), SerialConnection(256) {}

  void SendPacket(SimpleComputerPacket* p){
    p->InsertRange(0, 1);
    *p->Interpret(0) = p->id;    //Set first byte to be the id of the packet
    p->SeekStart();
    sc.Send(p);
  }

  void Receive(Packet* p) final { 
    sc.Receive(p); 
  }
};

CntrlCompConnection computer;
CntrlRadioConnection ms;
SimpleComputerPacket scp1 = SimpleComputerPacket(256);
CntrlRadioConnection cntrl;
RadioPacket rp1 = RadioPacket(256);

//Simple::Printf implementation stream to Rx
void radio_print(const char* fmt, ...){
  va_list args;
  va_start(args, fmt);
  
  scp1.config(PacketType::ComputerPrint);
  scp1.vPrintf((char*) fmt, args);
  computer.SendPacket(&scp1);

  va_end(args);
}

void setup(void) {
  Serial.begin(115200);
 
  while (!Serial)
    delay(10);

  if(!cntrl.Initialize(RF95_FREQ, RF95_POWER, Range::Medium)){
    Serial.printf("[Cntrl]: LoRa Radio Initialization Failed!\n");
    return;
  }
  printcntrlln("LoRa Okay!");

  //Listen to the ports
  computer.Start(); 
  cntrl.Start();
  printcntrlln("Initialization Complete!");
}

void loop() {
  Yield();
}

void CntrlSimpleCompConnection::Write(IO* io){ msc->Write(io); }

//Method called when a packet from the Feather Connection Pool is received
void CntrlRadioConnection::Receive(RadioPacket* p) {
  scp1.config(p->id);
  scp1.ReadFrom(*p);
  computer.SendPacket(&scp1);
}

//Method called when a packet from the computer is received
void CntrlSimpleCompConnection::ReceivedMessage(Packet* p){
  auto id = p->ReadByte();
  switch(id){
    
  } 
}