#include <SoftwareSerial.h>
#include "Adafruit_VL53L0X.h"


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

#define P_servo 5  //connectorMoter input1

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

#define P_lidarSDA A4
#define P_lidarSCL A5

#define P_batteryCheck A6  //battery positive pole wire
#define P_inspect A7       //Potentiometer wire
//----------------------------------------------------------------

//=================
#define stopcount 30  // 라인트레이서의 정지조건, timeoutCount가 얼마가 되면 멈추나?

#define minPulse 600
#define maxPulse 2400
#define period 20000
//------------------------


//=============현재 안씀=============================
SoftwareSerial BTSerial(7, 8);  // RX:7, TX:8 (RX는 HC06의 TX와 연결시킨다.)

byte maxdistancet = 150;
float timeOut = 2 * (maxdistancet + 10) / 100 / 340 * 1000000;
//===================================================


VL53L0X_RangingMeasurementData_t measure;  // Lidar 측정값
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

//================상태변수들================
int lineSensorInput;
int timeoutCount = 0;

//-------------차량 체크 값------------------
unsigned int inspectionVal = 0;
int trouble = 0;

float batteryVal = 0;
float batteryPercent = 0;

float distance;
int lastCarNum = 0;
//------------------------------------------
//===========================================


//=======================================
void setup() {
  Serial.begin(115200);
  BTSerial.begin(9600);
  delay(50);
  // wait until serial port opens for native USB devices
  while (!Serial) {
    delay(1);
  }

  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1)
      ;
  }

  pinMode(P_servo, OUTPUT);
  digitalWrite(P_servo, LOW);
  setServo(180);

  pinMode(P_FrontLED, OUTPUT);
  FrontLED_OFF;
}

//=======================================
void loop() {

  //while(!(BTSerial.available())) {}
  followLine();
  checkdistancetance();
  Serial.print("dis:"); Serial.println(distance);
  if (distance < 180 && distance > 100) {
    Serial.println("check!!");  //FORDEBUG
    delay(200);
    RCMove(0, 0);

    delay(5000);
  }

  // if (distance < 170 && distance > 100) {
  //   Serial.println("check!!");  //FORDEBUG
  //   delay(100);
  //   RCMove(0, 0);

  //   delay(1000);
  //   while (!checking()) {};  //4단자에 모두 접속할 떄까지 반복시도.
  //   Serial.print(inspectionVal); Serial.print(" ,"); Serial.println(batteryVal);  //FORDEBUG

  //   sendingData();

  //   distance = 100;
  //   //waitForStart();
  // }
}

void setServo(int degree) {
  int hTime = 0;
  int lTime = 0;

  if (degree < 0) degree = 0;
  if (degree > 180) degree = 180;

  hTime = (int)(minPulse + ((maxPulse - minPulse) / 180.0 * degree));
  lTime = period - hTime;

  digitalWrite(P_servo, HIGH);
  delayMicroseconds(hTime);
  digitalWrite(P_servo, LOW);
  delayMicroseconds(lTime);
}

void waitForStart() {
  while (!(Serial.available())) {}  //waiting until raspberrypi send ACK
  String data = Serial.readStringUntil('\n');
  delay(1000);
  // if((data.toInt())!=1){}
}

void sendingData() {
  String send_string = String(lastCarNum) + String(trouble) + String((int)(batteryPercent * 10));
  Serial.println(send_string);  //tranfer to raspberrypi

  delay(5000);
}

int checking() {
  int success = 0;
  int i;

  for (i = 180; i >= 10; i--) {
    setServo(i);
    delay(20);

    inspectionVal = (unsigned int)((unsigned int)(analogRead(P_inspect) * 100) / 512);
    batteryVal = (float)analogRead(P_batteryCheck) * 5 / 1024;
    Serial.print(inspectionVal);
    Serial.print(" ,");
    Serial.println(batteryVal);  //FORDEBUG

    FrontLED_ON;
    if (inspectionVal > 2 && batteryVal > 3) {
      if (i - 5 > 10) setServo(i - 5);
      delay(500);

      for (int i = 0; i < 150; i++) {
        inspectionVal = (unsigned int)((unsigned int)(analogRead(P_inspect) * 100) / 512);
        trouble = (inspectionVal > 50 ? 1 : 0);

        batteryVal = ((float)analogRead(P_batteryCheck) * 5 / 1024);
        if (batteryVal < 3.7)
          batteryPercent = 0;
        else
          batteryPercent = (batteryVal - 3.7) / 0.5 * 100;
        Serial.print(inspectionVal);
        Serial.print(" ,");
        Serial.println(batteryVal);  //FORDEBUG
        delay(300);
      }

      success = 1;
      lastCarNum++;
      break;
    }
  }

  delay(2000);

  for (; i < 180; i++) {
    setServo(i);
    delay(10);
  }

  delay(2000);
  FrontLED_OFF;

  return success;
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

void checkdistancetance() {
  lox.rangingTest(&measure, false);  // pass in 'true' to get debug data printout!
  if (measure.RangeStatus != 4) {    // phase failures have incorrect data
    distance = measure.RangeMilliMeter;
  }
}

void followLine() {
  lineSensorInput = PINC &= B00000111;  //Right:centerk:left

  switch (lineSensorInput) {
    //공중에 떠있음
    case 0b000: RCMove(80, 80); break;

    //라인을 벗어남
    case 0b111:
      if (timeoutCount > stopcount)
        RCMove(0, 0);
      else {
        RCMove(80, 80);
        timeoutCount += 1;
      }
      break;

    //중앙 위치
    case 0b101:
      RCMove(80, 80);
      timeoutCount = 0;
      break;

    //오른쪽으로 감
    case 0b100:
    case 0b110:
      RCMove(-90, 120);
      timeoutCount = 0;
      break;

    //왼쪽으로 감
    case 0b001:
    case 0b011:
      RCMove(120, -90);
      timeoutCount = 0;
      break;
  }
}
