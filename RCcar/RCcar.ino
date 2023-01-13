#include <SoftwareSerial.h>
// RX:7, TX:8 (RX는 HC06의 TX와 연결시킨다.)
SoftwareSerial BTSerial(7, 8);

//==================pre-made macro function=========================
#define LmotorF(speed) \
  { \
    analogWrite(P_motorL1, speed); \
    analogWrite(P_motorL2, 0); \
  }
#define LmotorB(speed) \
  { \
    analogWrite(P_motorL1, 0); \
    analogWrite(P_motorL2, speed); \
  }
#define RmotorF(speed) \
  { \
    analogWrite(P_motorR1, 0); \
    analogWrite(P_motorR2, speed); \
  }
#define RmotorB(speed) \
  { \
    analogWrite(P_motorR1, speed); \
    analogWrite(P_motorR2, 0); \
  }

#define Line_stop \
  { \
    LmotorF(0); \
    RmotorF(0); \
  }
#define Line_Forward \
  { \
    LmotorF(LFspeed); \
    RmotorF(LFspeed); \
  }
#define Line_Rtankturn \
  { \
    LmotorF(LTspeed); \
    RmotorB(LTspeed); \
  }
#define Line_Ltankturn \
  { \
    LmotorB(LTspeed); \
    RmotorF(LTspeed); \
  }

#define FrontLED_ON \
  { digitalWrite(P_FrontLED, HIGH); }
#define FrontLED_OFF \
  { digitalWrite(P_FrontLED, LOW); }

#define Hspeed 255  // 고속
#define Lspeed 90   // 저속
#define Tspeed 150  // 탱크턴에 맞는 속도

#define LFspeed 90   // 라인트레이서의 직진 속도
#define LTspeed 255  // 라인트레이서의 회전 속도
//----------------------------------------------------------------

//==============port definition=================================
#define P_SerialRx 0  //mini type-B USB Rx
#define P_SerialTx 1  //mini type-B USB Tx

#define P_trig 2  //ultrawave trig
#define P_echo 4  //ultrawave echo

#define P_connectMoter1 5  //connectorMoter input1
#define P_connectMoter2 6  //connectorMoter input2

#define P_BTSerialRx 7  //bluetoothRx
#define P_BTSerialTx 8  //bluetoothTx

#define P_motorR1 9   // rightmotor input1
#define P_motorR2 10  // rightmotor input2
#define P_motorL1 11  // leftmotor input1
#define P_motorL2 3   // leftmotor input2

#define P_FrontLED 12  //headlight

#define P_lineSensorL A0  //left sensor of line follower sensor
#define P_lineSensorC A1  //centor sensor of line follower
#define P_lineSensorR A2  //right sensor of line follower sensor

#define P_inspect A3       //Potentiometer wire
#define P_batteryCheck A4  //battery positive pole wire
//----------------------------------------------------------------

//=================
#define stopcount 15000  // 라인트레이서의 정지조건, zerocount가 얼마가 되면 멈추나?

//------------------------
// 상태변수들
int lineSensorInput;
int zerocount = 0;

int parking_lot = 1 ;
int troubleRatio;
int batteryVoltage;

int send_flag = 0 ;
int drive_flag = 1 ;

//=======================================
void RCMove(int LS, int RS);

void setup() {
  Serial.begin(9600);
  // BTSerial.begin(9600);

  pinMode(P_trig, OUTPUT);
  pinMode(P_echo, INPUT);

  pinMode(P_connectMoter1, OUTPUT);
  pinMode(P_connectMoter2, OUTPUT);
  analogWrite(P_connectMoter1, 0);
  analogWrite(P_connectMoter2, 0);


  pinMode(P_motorR1, OUTPUT);
  pinMode(P_motorR2, OUTPUT);
  pinMode(P_motorL1, OUTPUT);
  pinMode(P_motorL2, OUTPUT);

  pinMode(P_FrontLED, OUTPUT);
  FrontLED_OFF;

  //@@@analogRead port doesn't need configuration

  // BTSerial.print("*RR255G0B0*");
  // BTSerial.print("*YR0G0B0*");
  // BTSerial.print("*GR0G0B*");
}

//=======================================
float distance;

void loop() {  
  
  if(Serial.available() > 0 && drive_flag==0){
    String data = Serial.readStringUntil('\n');
    Serial.println(data);
    delay(1000);
    if((data.toInt())!=1){
      return;
    }
    drive_flag = 1;
  }


  if (drive_flag == 1){
  
    followLine();
    distance = getDistance();
    
    if(distance<15) {
      delay(100);
      drive_flag=0;
    }
    
  }    

  else if(drive_flag ==0) {
    
      RCMove(0,0);
      checking(); // get troubleratio, batteryVoltage

      String send_string = String(parking_lot)+String(troubleRatio)+String(batteryVoltage);
      Serial.println(send_string);
      
    }
    
  
}

void checking() {
  
  parking_lot ++;
  parking_lot = parking_lot % 3;
  troubleRatio = 1;
  batteryVoltage = 50;
  
}

float getDistance() {  
  float duration, distance;

  digitalWrite(P_trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(P_trig, LOW);

  duration = pulseIn(P_echo, HIGH);  //물체에 반사되어돌아온 초음파의 시간을 변수에 저장합니다.
  distance = duration * 17 /1000;   //duration[us] * (1s/1,000,000us) * 340m/s * (100cm/1m) / 2 => 17 / 1000 [cm]
  
  return distance;
}

void followLine() {
  lineSensorInput = PINC &= B00000111;  //Right:centerk:left

  switch (lineSensorInput) {
    //공중에 떠있음
    case 0b000: RCMove(0, 0); break;

    //라인을 벗어남
    case 0b111:
      if (zerocount > stopcount)
        RCMove(0, 0);
      else {
        RCMove(90, 90);
        zerocount += 1;
      }
      break;

    //중앙 위치
    case 0b101:
      RCMove(90, 90);
      zerocount = 0;
      break;

    //오른쪽으로 감
    case 0b100:
    case 0b110:
      RCMove(7, 70);
      zerocount = 0;
      break;

    //왼쪽으로 감
    case 0b001:
    case 0b011:
      RCMove(70, 7);
      zerocount = 0;
      break;
  }
}

void RCMove(int LS, int RS) {
  if (LS >= 0) {
    analogWrite(P_motorL1, LS);
    analogWrite(P_motorL2, 0);
  } else {
    analogWrite(P_motorL1, 0);
    analogWrite(P_motorL2, -LS);
  }

  if (RS >= 0) {
    analogWrite(P_motorR2, RS);
    analogWrite(P_motorR1, 0);
  } else {
    analogWrite(P_motorR2, 0);
    analogWrite(P_motorR1, -RS);
  }
}
