#ifndef MOTOR_H
#define MOTOR_H

#define M1Enable 6
#define M1Dir1 9
#define M1Dir2 10

#define M2Enable 13
#define M2Dir1 11
#define M2Dir2 12

#define MotorMinPower .4

void writeMotorValue(int enablePin, int dir1Pin, int dir2Pin, float fraction){
  analogWrite(enablePin, 255 * fraction);
  bool dir1Value = false;
  bool dir2Value = false;

  if(fraction > 0){
    dir1Value = true;
  }else if(fraction < 0){
    dir2Value = true;
  }

  digitalWrite(dir1Pin, dir1Value);
  digitalWrite(dir2Pin, dir2Value);
}

void init_motors(){
  pinMode(M1Enable, OUTPUT);
  pinMode(M1Dir1, OUTPUT);
  pinMode(M1Dir2, OUTPUT);
  pinMode(M2Enable, OUTPUT);
  pinMode(M2Dir1, OUTPUT);
  pinMode(M2Dir2, OUTPUT);
}

#endif