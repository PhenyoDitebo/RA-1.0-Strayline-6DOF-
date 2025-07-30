#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>
#include <AccelStepper.h>

/* ---------------- Bluetooth & PCA9685 ---------------- */
SoftwareSerial BTserial(2, 3);             // HC-05 RX,TX
Adafruit_PWMServoDriver pwm;
const int SERVOMIN = 150, SERVOMAX = 600, SERVO_FREQ = 50;

/* Servo channels */
const int MIRROR   = 0;
const int SHOULDER = 1;
const int FLICK    = 2;    // wrist tilt (W:)
const int ELBOW    = 3;
const int ROTATE   = 4;    // gripper roll
const int GRIPPER  = 5;

/* ---------------- Step/Dir driver ---------------- */
const int STEP_PIN = 4;
const int DIR_PIN  = 5;
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

/* Live & target servo angles */
int curS=0 , tgtS=0 ;  bool shoulderActive=false;
int curM=180,tgtM=180;
int curE=70 , tgtE=100;
int curF=65 , tgtF=60;
int curR=170,tgtR=170;
int curG=90 , tgtG=90;

const int STEP_DEG = 1;
const int UPD_INT  = 15;
unsigned long lastUpd = 0;

/* Base-speed mapping */
const int  DEAD     = 10;      // ±deg neutral
const float SCALE   = 3.0;     // bigger = gentler
const float MAX_SPS = 1200;    // ≈120 RPM @ 200 spr

/* ------------ helpers ------------ */
int pulse(int a){ return map(a,0,180,SERVOMIN,SERVOMAX); }

int getAngle(const String& s,const char* tag,int cur){
  int i=s.indexOf(tag);  if(i==-1) return cur;
  int j=s.indexOf('|',i);
  String v=(j==-1? s.substring(i+2):s.substring(i+2,j));
  return constrain(v.toInt(),0,180);
}

void smooth(int ch,int &cur,int tgt){
  if(abs(tgt-cur)<=STEP_DEG) cur=tgt;
  else cur += (tgt>cur?STEP_DEG:-STEP_DEG);
  pwm.setPWM(ch,0,pulse(cur));
}

/* ------------ base turntable from B: ------------ */
void updateBase(int B){           // 0-180
  int d = B - 90;                 // −90…+90
  if(abs(d) < DEAD){              // stop zone
    stepper.setSpeed(0);
    return;
  }
  float sps = constrain(abs(d)/SCALE, 0, MAX_SPS);
  stepper.setSpeed( d>0 ?  +sps : -sps );
}

void setup(){
  Serial.begin(9600);
  BTserial.begin(9600);

  pwm.begin(); pwm.setPWMFreq(SERVO_FREQ);
  pwm.setPWM(ELBOW  ,0,pulse(curE));
  pwm.setPWM(FLICK  ,0,pulse(curF));
  pwm.setPWM(ROTATE ,0,pulse(curR));
  pwm.setPWM(GRIPPER,0,pulse(curG));

  stepper.setMaxSpeed(MAX_SPS);
  stepper.setAcceleration(200);        // soft ramp
  /* ENA- on driver must be tied to GND */

  Serial.println("Receiver v8.2  – 6-DOF servos + AccelStepper base");
}

void loop(){
  /* ------------ Bluetooth packet ------------ */
  if(BTserial.available()){
    String in = BTserial.readStringUntil('\n');
    Serial.println(in);

    tgtS = getAngle(in,"S:",tgtS);      // shoulders
    tgtM = constrain(180-tgtS,30,150);
    shoulderActive = true;

    tgtE = constrain(getAngle(in,"E:",tgtE),75,180);
    tgtG = getAngle(in,"G:",tgtG);
    tgtR = getAngle(in,"R:",tgtR);
    tgtF = getAngle(in,"W:",tgtF);      // wrist flick (W:)

    int B = getAngle(in,"B:",90);       // base turntable (B:)
    updateBase(B);
  }

  /* continuous base motor drive */
  stepper.runSpeed();

  /* smooth servos */
  if(millis()-lastUpd >= UPD_INT){
    lastUpd = millis();
    if(shoulderActive){
      smooth(SHOULDER,curS,tgtS);
      smooth(MIRROR  ,curM,tgtM);
    }
    smooth(ELBOW  ,curE,tgtE);
    smooth(FLICK  ,curF,tgtF);
    smooth(ROTATE ,curR,tgtR);
    smooth(GRIPPER,curG,tgtG);
  }
}
