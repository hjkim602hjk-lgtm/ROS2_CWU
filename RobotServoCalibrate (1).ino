// NUCLEO-F446RE / STM32duino 3.0.0. Robot40cm V3.
// Same pins, motor polarity, Kp, PSD threshold and servo settings as supplied.
// Encoder counting: A-channel RISING only. Calibrate THIS firmware's counts.
#include <Arduino.h>
#include <Servo.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// Servo settings (was ServoSettings.h, not committed). START/GRIP from this file's help text.
// Calibrate with [L105]/[R095], then copy the found angles here.
constexpr int SERVO_LEFT_START=70, SERVO_RIGHT_START=130;
constexpr int SERVO_LEFT_GRIP=130, SERVO_RIGHT_GRIP=70;
// ponytail: full 0~180 while the gripper's mechanical limits are unknown; narrow once measured.
constexpr int SERVO_LEFT_MIN=0, SERVO_LEFT_MAX=180, SERVO_RIGHT_MIN=0, SERVO_RIGHT_MAX=180;

Uart PiSerial(PC_11, PC_10); // RX, TX
constexpr uint32_t LEFT_PWM=D6, LEFT_DIR=D7, RIGHT_DIR=D8, RIGHT_PWM=PA7_ALT3;
constexpr uint32_t LEFT_ENC_A=D4, LEFT_ENC_B=D5, RIGHT_ENC_A=D9, RIGHT_ENC_B=D10;
constexpr uint32_t PSD_FRONT=A0, PSD_REAR=A1, PSD_R=A2, PSD_L=A3;
constexpr bool INVERT_LEFT_MOTOR=true, INVERT_RIGHT_MOTOR=false;
constexpr int LEFT_ENCODER_SIGN=-1, RIGHT_ENCODER_SIGN=1;
constexpr int BASE_PWM=80, MAX_PWM=130, TURN_PWM=90, PSD_THRESHOLD=550;
constexpr float Kp=0.30f, Ki=0.0f, Kd=0.0f, MAX_CORRECTION=30.0f;
constexpr uint32_t CONTROL_MS=50, MOTOR_TEST_MS=1000, TELEMETRY_MS=50;
constexpr uint32_t HEARTBEAT_TIMEOUT_MS=700, ENCODER_TIMEOUT_MS=350;
constexpr uint32_t DISTANCE_TIMEOUT_MS=15000, CONTINUOUS_LIMIT_MS=10000;
constexpr uint32_t SETTLE_MS=300, SERVO_STEP_MS=20;
// Motor start noise spikes PSD over threshold for <1 sample; require it to persist.
constexpr uint32_t PSD_HOLD_MS=20;
constexpr float DISTANCE_MM=400.0f, MAX_WHEEL_MISMATCH_MM=40.0f;
// Do not invent counts/mm. Pi restores calibrated values using [Kleft,right].
float countsPerMmL=0.0f, countsPerMmR=0.0f;

enum RunMode { MODE_STOP=0, MODE_LEFT_TEST=1, MODE_RIGHT_TEST=2,
  MODE_BOTH_TEST=3, MODE_PID_TEST=4, MODE_PID_CONTINUOUS=5,
  MODE_AUTO_PSD=6, MODE_DISTANCE=7, MODE_DRIVE=8 };
struct Parser { char buf[48]; uint8_t len; bool active; bool bad; };
// Explicit prototypes avoid Arduino auto-prototype ordering for custom types.
bool startMode(RunMode mode, uint8_t source);
void receiveByte(char c, Parser &parser, uint8_t source);

Parser parsers[2]={};
RunMode currentMode=MODE_STOP;
Servo servoLeft, servoRight;
int servoLeftAngle=SERVO_LEFT_START, servoRightAngle=SERVO_RIGHT_START;
int servoLeftTarget=SERVO_LEFT_START, servoRightTarget=SERVO_RIGHT_START;
volatile int32_t leftEncoderCount=0, rightEncoderCount=0;
int currentLeftPWM=0, currentRightPWM=0, targetLeftPWM=0, targetRightPWM=0;
int lastSignL=0, lastSignR=0;
uint32_t zeroTimeL=0, zeroTimeR=0;
uint32_t lastMotorTime=0, lastServoTime=0, lastControlTime=0, lastTelemetryTime=0;
uint32_t modeStartTime=0, lastPulseL=0, lastPulseR=0;
uint32_t frontOverSince=0, pivotOverSince=0;
uint32_t heartbeatTime[2]={0,0}; bool heartbeatSeen[2]={false,false};
uint8_t motionOwner=0, calibrationOwner=0;
int32_t startL=0, startR=0, previousL=0, previousR=0, watchedL=0, watchedR=0;
float integral=0, previousError=0;
int wrongCycles=0;
// w/e: l/r wheel test without the 1 s limit (heartbeat + 10 s cap) for tick counting.
bool wheelHold=false;
// [Dl,r]: Pi-commanded signed PWM (ROS cmd_vel), heartbeat required, no time cap.
int driveL=0, driveR=0;
// [Vl,r]: wheel speed targets in ticks/s, forward = + ; per-wheel PI every CONTROL_MS.
// Gains come from the Pi YAML via [Gff,static,kp,ki] (RAM only, resent by the Pi).
// Raw V3 ticks fall when the robot drives forward (2026-10-11), hence the -1 signs.
constexpr int FWD_SIGN_L=-1, FWD_SIGN_R=-1;
constexpr float MAX_TPS=1000.0f;
bool velMode=false, gainsSet=false;
float gFF=0, gStatic=0, gKp=0, gKi=0;
float velTarget[2]={0,0}, velInt[2]={0,0};
int velPwm[2]={0,0};
int32_t velPrevL=0, velPrevR=0;
uint32_t velTime=0;
bool telemetryStream=false, helpPending=false, statusPending=false;
bool calibrating=false, settling=false;
uint32_t calibrationTime=0, settleStart=0;
int32_t calibrationL=0, calibrationR=0;
float distanceL=0, distanceR=0;
bool reachedL=false, reachedR=false;

void snapshot(int32_t &l,int32_t &r) {
  noInterrupts(); l=leftEncoderCount; r=rightEncoderCount; interrupts();
}
void leftEncoderISR() { leftEncoderCount += digitalRead(LEFT_ENC_B) ? LEFT_ENCODER_SIGN : -LEFT_ENCODER_SIGN; }
void rightEncoderISR() { rightEncoderCount += digitalRead(RIGHT_ENC_B) ? RIGHT_ENCODER_SIGN : -RIGHT_ENCODER_SIGN; }
void say(const char *message) { PiSerial.println(message); Serial.println(message); }

void stopMotors() {
  analogWrite(LEFT_PWM,0); analogWrite(RIGHT_PWM,0);
  if(currentLeftPWM) zeroTimeL=millis();
  if(currentRightPWM) zeroTimeR=millis();
  currentLeftPWM=currentRightPWM=targetLeftPWM=targetRightPWM=0;
}
void halt(const char *reason) {
  stopMotors(); currentMode=MODE_STOP; settling=false; wheelHold=false;
  servoLeftTarget=servoLeftAngle; servoRightTarget=servoRightAngle;
  say(reason);
}
void driveOne(uint32_t pwm,uint32_t dir,int value,bool invert) {
  if(value==0) { analogWrite(pwm,0); return; }
  digitalWrite(dir,((value>0)^invert)?HIGH:LOW);
  analogWrite(pwm,abs(value));
}
int ramp(int value,int target,int &lastSign,uint32_t &zeroTime,uint32_t now) {
  int sign=(target>0)-(target<0);
  if(value && sign && ((value>0)!=(target>0))) target=0;
  if(!value && sign && lastSign && sign!=lastSign && now-zeroTime<50) return 0;
  int next=value;
  if(next<target) next+=min(8,target-next);
  else if(next>target) next-=min(8,next-target);
  if(!next && value) zeroTime=now;
  if(next) lastSign=(next>0)?1:-1;
  return next;
}
void setMotors(int l,int r) {
  targetLeftPWM=constrain(l,-MAX_PWM,MAX_PWM);
  targetRightPWM=constrain(r,-MAX_PWM,MAX_PWM);
}
void updateMotors() {
  uint32_t now=millis(); if(now-lastMotorTime<10) return;
  lastMotorTime=now;
  currentLeftPWM=ramp(currentLeftPWM,targetLeftPWM,lastSignL,zeroTimeL,now);
  currentRightPWM=ramp(currentRightPWM,targetRightPWM,lastSignR,zeroTimeR,now);
  driveOne(LEFT_PWM,LEFT_DIR,currentLeftPWM,INVERT_LEFT_MOTOR);
  driveOne(RIGHT_PWM,RIGHT_DIR,currentRightPWM,INVERT_RIGHT_MOTOR);
}
bool servoBusy() { return servoLeftAngle!=servoLeftTarget || servoRightAngle!=servoRightTarget; }
void updateServos() {
  uint32_t now=millis(); if(now-lastServoTime<SERVO_STEP_MS) return;
  lastServoTime=now;
  if(servoLeftAngle!=servoLeftTarget) {
    servoLeftAngle+=(servoLeftTarget>servoLeftAngle)?1:-1;
    servoLeft.write(servoLeftAngle);
  }
  if(servoRightAngle!=servoRightTarget) {
    servoRightAngle+=(servoRightTarget>servoRightAngle)?1:-1;
    servoRight.write(servoRightAngle);
  }
}
bool calibrated() { return countsPerMmL>0 && countsPerMmR>0; }
bool needsHeartbeat() {
  return currentMode==MODE_DISTANCE || currentMode==MODE_PID_CONTINUOUS || currentMode==MODE_AUTO_PSD || currentMode==MODE_DRIVE || wheelHold;
}
bool startMode(RunMode mode,uint8_t source) {
  if(currentMode!=MODE_STOP || settling || calibrating || servoBusy()) { say("# REJECT BUSY"); return false; }
  if(mode==MODE_DISTANCE && !calibrated()) { say("# REJECT CALIBRATE_FIRST"); return false; }
  if(mode==MODE_DISTANCE || mode==MODE_PID_CONTINUOUS || mode==MODE_AUTO_PSD || mode==MODE_DRIVE || wheelHold) {
    if(!heartbeatSeen[source] || millis()-heartbeatTime[source]>HEARTBEAT_TIMEOUT_MS) {
      say("# REJECT HEARTBEAT_REQUIRED"); return false;
    }
  }
  stopMotors(); motionOwner=source;
  snapshot(startL,startR); previousL=watchedL=startL; previousR=watchedR=startR;
  integral=previousError=0; wrongCycles=0; distanceL=distanceR=0;
  reachedL=reachedR=false;
  modeStartTime=lastControlTime=lastPulseL=lastPulseR=millis();
  frontOverSince=pivotOverSince=0; driveL=driveR=0;
  velTarget[0]=velTarget[1]=velInt[0]=velInt[1]=0; velPwm[0]=velPwm[1]=0;
  snapshot(velPrevL,velPrevR); velTime=millis();
  currentMode=mode; say("# START"); return true;
}
void reportDistance(Stream &out) {
  int32_t l,r; snapshot(l,r);
  out.print("# DIST_MM,"); out.print((float)(l-startL)/countsPerMmL,2);
  out.print(','); out.println((float)(r-startR)/countsPerMmR,2);
}
// Every loop checks end position, independently of the 50 ms PID period.
void checkDistance() {
  int32_t l,r; snapshot(l,r);
  distanceL=(float)(l-startL)/countsPerMmL; distanceR=(float)(r-startR)/countsPerMmR;
  if(fabsf(distanceL-distanceR)>MAX_WHEEL_MISMATCH_MM) { halt("# ABORT WHEEL_MISMATCH"); return; }
  reachedL=reachedL || distanceL>=DISTANCE_MM;
  reachedR=reachedR || distanceR>=DISTANCE_MM;
  if(reachedL) { analogWrite(LEFT_PWM,0); currentLeftPWM=targetLeftPWM=0; }
  if(reachedR) { analogWrite(RIGHT_PWM,0); currentRightPWM=targetRightPWM=0; }
  if(reachedL && reachedR) {
    stopMotors(); currentMode=MODE_STOP; settling=true; settleStart=millis();
    say("# TARGET_REACHED");
  }
}
void runPID() {
  uint32_t now=millis(), elapsed=now-lastControlTime;
  if(elapsed<CONTROL_MS) return;
  int32_t l,r; snapshot(l,r);
  int32_t dl=l-previousL, dr=r-previousR;
  previousL=l; previousR=r; lastControlTime=now;
  // Preserve original Kp/count units; distance mode corrects unequal wheel scale.
  float rightEquivalent=(float)dr;
  if(currentMode==MODE_DISTANCE) rightEquivalent*=countsPerMmL/countsPerMmR;
  float error=((float)dl-rightEquivalent)*CONTROL_MS/elapsed;
  float seconds=elapsed/1000.0f;
  integral=constrain(integral+error*seconds,-100.0f,100.0f);
  float correction=constrain(Kp*error+Ki*integral+Kd*(error-previousError)/seconds,
                             -MAX_CORRECTION,MAX_CORRECTION);
  previousError=error;
  int base=BASE_PWM;
  if(currentMode==MODE_DISTANCE) {
    float remaining=DISTANCE_MM-min(distanceL,distanceR);
    // Gentle slowdown only: avoid assuming that low PWM can overcome friction.
    if(remaining<60.0f) base=70;
    if(reachedL || reachedR) correction=0;
  }
  int pl=constrain(base-(int)correction,0,MAX_PWM);
  int pr=constrain(base+(int)correction,0,MAX_PWM);
  if(currentMode==MODE_DISTANCE) { if(reachedL) pl=0; if(reachedR) pr=0; }
  setMotors(pl,pr);
  if(dl<0 || dr<0) ++wrongCycles; else wrongCycles=0;
  if(wrongCycles>=3) halt("# ABORT ENCODER_DIRECTION");
}
void checkStall() {
  int32_t l,r; snapshot(l,r); uint32_t now=millis();
  if(l!=watchedL || currentLeftPWM==0) lastPulseL=now;
  if(r!=watchedR || currentRightPWM==0) lastPulseR=now;
  watchedL=l; watchedR=r;
  if((currentLeftPWM!=0 && now-lastPulseL>=ENCODER_TIMEOUT_MS) ||
     (currentRightPWM!=0 && now-lastPulseR>=ENCODER_TIMEOUT_MS)) halt("# ABORT ENCODER_NO_PULSE");
}
// True once `over` has held for PSD_HOLD_MS; since=0 means currently clear.
bool psdHeld(bool over,uint32_t &since,uint32_t now) {
  if(!over) { since=0; return false; }
  if(!since) since=now;
  return now-since>=PSD_HOLD_MS;
}
void runPSDAuto() {
  int f=analogRead(PSD_FRONT), b=analogRead(PSD_REAR), l=analogRead(PSD_L), r=analogRead(PSD_R);
  if(f<PSD_THRESHOLD && l<PSD_THRESHOLD && r<PSD_THRESHOLD) { setMotors(BASE_PWM,BASE_PWM); return; }
  if(b>=PSD_THRESHOLD || (l>=PSD_THRESHOLD && r>=PSD_THRESHOLD)) { halt("# ABORT PSD_TURN_BLOCKED"); return; }
  if(f>=PSD_THRESHOLD) { if(l<r) setMotors(-TURN_PWM,TURN_PWM); else setMotors(TURN_PWM,-TURN_PWM); }
  else if(l>=PSD_THRESHOLD) setMotors(TURN_PWM,-TURN_PWM);
  else setMotors(-TURN_PWM,TURN_PWM);
}
void reportCalibration(Stream &out) {
  out.print("# CAL,"); out.print(countsPerMmL,6); out.print(','); out.println(countsPerMmR,6);
}
bool validScale(float l,float r) {
  return isfinite(l) && isfinite(r) && l>=0.01f && r>=0.01f && l<=10000 && r<=10000 && l/r>=0.5f && l/r<=2.0f;
}
// [D] PWM or [V] ticks/s; false = malformed or out of range.
bool driveFrame(char *s,uint8_t source) {
  bool vel=s[0]=='V';
  char *end; long l=strtol(s+1,&end,10);
  if(end==s+1 || *end!=',') return false;
  char *next=end+1; long r=strtol(next,&end,10);
  long limit=vel?(long)MAX_TPS:MAX_PWM;
  if(end==next || *end || labs(l)>limit || labs(r)>limit) return false;
  if(vel && !gainsSet) { say("# REJECT GAINS_FIRST"); return true; }
  if(currentMode==MODE_STOP) {
    if(!startMode(MODE_DRIVE,source)) return true;
  } else if(currentMode!=MODE_DRIVE || source!=motionOwner) { say("# REJECT BUSY"); return true; }
  velMode=vel;
  if(vel) { velTarget[0]=l; velTarget[1]=r; } else { driveL=l; driveR=r; }
  return true;
}
bool gainFrame(char *s) {
  float v[4]; char *p=s+1, *end;
  for(int i=0;i<4;++i) {
    v[i]=strtof(p,&end);
    if(end==p || !isfinite(v[i]) || v[i]<0 || v[i]>1000) return false;
    if(i<3 && *end!=',') return false;
    p=end+1;
  }
  if(*end) return false;
  gFF=v[0]; gStatic=v[1]; gKp=v[2]; gKi=v[3]; gainsSet=true;
  return true;
}
// One PI step per wheel; target 0 means output 0 and a cleared integrator.
void velocityControl(uint32_t now) {
  if(now-velTime<CONTROL_MS) return;
  float dt=(now-velTime)*0.001f; velTime=now;
  int32_t l,r; snapshot(l,r);
  float speed[2]={(float)(l-velPrevL)*FWD_SIGN_L/dt,(float)(r-velPrevR)*FWD_SIGN_R/dt};
  velPrevL=l; velPrevR=r;
  for(int i=0;i<2;++i) {
    float target=velTarget[i];
    if(target==0) { velInt[i]=0; velPwm[i]=0; continue; }
    float err=target-speed[i];
    float base=gFF*target+(target>0?gStatic:-gStatic)+gKp*err;
    float candidate=constrain(velInt[i]+gKi*err*dt,-(float)MAX_PWM,(float)MAX_PWM);
    float out=base+candidate;
    // Conditional integration: do not wind up further into saturation.
    if(fabsf(out)<=MAX_PWM || (out>0)!=(err>0)) velInt[i]=candidate;
    velPwm[i]=(int)lroundf(constrain(base+velInt[i],-(float)MAX_PWM,(float)MAX_PWM));
  }
}
void frameCommand(char *s,uint8_t source) {
  // Drive frames stream at 20 Hz while moving, so they bypass the STOP-only check.
  if(s[0]=='D' || s[0]=='V') { if(!driveFrame(s,source)) say("# REJECT DRIVE_FORMAT"); return; }
  if(s[0]=='G') { say(gainFrame(s)?"# GAINS_OK":"# REJECT GAIN_FORMAT"); return; }
  if(currentMode!=MODE_STOP || settling || servoBusy()) { say("# REJECT BUSY"); return; }
  if((s[0]=='L' || s[0]=='R') && strlen(s)==4) {
    for(int i=1;i<4;++i) if(s[i]<'0' || s[i]>'9') { say("# REJECT SERVO_FORMAT"); return; }
    int angle=atoi(s+1);
    int lo=s[0]=='L'?SERVO_LEFT_MIN:SERVO_RIGHT_MIN, hi=s[0]=='L'?SERVO_LEFT_MAX:SERVO_RIGHT_MAX;
    if(calibrating || angle<lo || angle>hi) { say("# REJECT SERVO_RANGE_OR_CAL"); return; }
    if(s[0]=='L') servoLeftTarget=angle; else servoRightTarget=angle;
    say("# SERVO_TARGET"); return;
  }
  char *end;
  if(s[0]=='C') {
    float mm=strtof(s+1,&end);
    if(!calibrating || source!=calibrationOwner || end==s+1 || *end || !isfinite(mm) || mm<100 || mm>2000) {
      say("# REJECT CAL_FORMAT_OR_STATE"); return;
    }
    int32_t l,r; snapshot(l,r); l-=calibrationL; r-=calibrationR;
    if(l<20 || r<20 || !validScale(l/mm,r/mm)) { say("# REJECT CAL_COUNTS_OR_DIRECTION"); return; }
    countsPerMmL=l/mm; countsPerMmR=r/mm; calibrating=false;
    reportCalibration(PiSerial); reportCalibration(Serial); return;
  }
  if(s[0]=='K') {
    float l=strtof(s+1,&end);
    if(calibrating || end==s+1 || *end!=',') { say("# REJECT SCALE_FORMAT"); return; }
    char *next=end+1; float r=strtof(next,&end);
    if(end==next || *end || !validScale(l,r)) { say("# REJECT SCALE_RANGE"); return; }
    countsPerMmL=l; countsPerMmR=r;
    reportCalibration(PiSerial); reportCalibration(Serial); return;
  }
  say("# REJECT FRAME");
}
void command(char c,uint8_t source) {
  if(c=='\r' || c=='\n') return;
  if(c=='!') { heartbeatSeen[source]=true; heartbeatTime[source]=millis(); return; }
  if(c=='q' || c==' ') { calibrating=false; halt("# STOP"); return; }
  if(c=='h') { helpPending=true; return; }
  if(c=='s') { statusPending=true; return; }
  if(c=='t') { telemetryStream=!telemetryStream; say(telemetryStream?"Telemetry = ON":"Telemetry = OFF"); return; }
  if(c=='l') { startMode(MODE_LEFT_TEST,source); return; }
  if(c=='r') { startMode(MODE_RIGHT_TEST,source); return; }
  if((c=='w' || c=='e') && currentMode==MODE_STOP) {
    wheelHold=true;
    if(!startMode(c=='w'?MODE_LEFT_TEST:MODE_RIGHT_TEST,source)) wheelHold=false;
    return;
  }
  if(c=='b') { startMode(MODE_BOTH_TEST,source); return; }
  if(c=='p') { startMode(MODE_PID_TEST,source); return; }
  if(c=='P') { startMode(MODE_PID_CONTINUOUS,source); return; }
  if(c=='a') { startMode(MODE_AUTO_PSD,source); return; }
  if(c=='f') { startMode(MODE_DISTANCE,source); return; }
  if(c=='z') {
    calibrating=false; halt("# STOP");
    noInterrupts(); leftEncoderCount=rightEncoderCount=0; interrupts();
    say("# ENCODERS_RESET"); return;
  }
  if(currentMode!=MODE_STOP || settling || servoBusy()) { say("# REJECT BUSY"); return; }
  if(c=='c') {
    stopMotors(); snapshot(calibrationL,calibrationR); calibrating=true;
    calibrationOwner=source; calibrationTime=millis(); say("# CAL_BEGIN PUSH_FORWARD_400MM_THEN_[C400]"); return;
  }
  if(calibrating) { say("# REJECT CAL_ACTIVE"); return; }
  switch(c) {
    case '1': servoLeftTarget=30; break;
    case '2': servoLeftTarget=60; break;
    case '3': servoLeftTarget=90; break;
    case '4': servoRightTarget=30; break;
    case '5': servoRightTarget=60; break;
    case '6': servoRightTarget=90; break;
    case '7': servoLeftTarget=SERVO_LEFT_START; servoRightTarget=SERVO_RIGHT_START; break;
    case '8': servoLeftTarget=SERVO_LEFT_GRIP; servoRightTarget=SERVO_RIGHT_GRIP; break;
    default: say("# REJECT COMMAND"); break;
  }
}
void receiveByte(char c,Parser &p,uint8_t source) {
  // STOP is immediate even inside an incomplete frame. Discard its remainder.
  if(c=='q' || c==' ') { command(c,source); if(p.active) p.bad=true; return; }
  if(c=='[') { p.active=true; p.bad=false; p.len=0; return; }
  if(!p.active) { command(c,source); return; }
  if(c=='\r' || c=='\n') { p.active=false; p.len=0; return; }
  if(c==']') {
    p.active=false; p.buf[p.len]='\0';
    if(p.bad || p.len==0) say("# REJECT FRAME"); else frameCommand(p.buf,source);
    p.len=0; return;
  }
  if(p.len<sizeof(p.buf)-1 && !p.bad) p.buf[p.len++]=c; else p.bad=true;
}
void printTelemetry(Stream &out) {
  int32_t l,r; snapshot(l,r);
  out.print(millis()); out.print(','); out.print((int)currentMode);
  out.print(','); out.print(l); out.print(','); out.print(r);
  out.print(','); out.print(currentLeftPWM); out.print(','); out.print(currentRightPWM);
  out.print(','); out.print(analogRead(PSD_FRONT)); out.print(','); out.print(analogRead(PSD_REAR));
  out.print(','); out.print(analogRead(PSD_L)); out.print(','); out.print(analogRead(PSD_R));
  out.print(','); out.print(servoLeftAngle); out.print(','); out.println(servoRightAngle);
}
void help(Stream &out) {
  out.println("# ROBOT40_V3");
  out.println("# f: 400mm PID; needs calibration and ! heartbeat every 200ms");
  out.println("# c: manual calibration begin; push forward then [C400]");
  out.println("# [Kleft,right]: restore counts/mm (RAM only)");
  out.println("# l/r/b/p: left/right/both/PID 1 sec; l/r are NOT turns");
  out.println("# w/e: left/right wheel until q, heartbeat required, max 10 sec");
  out.println("# [D80,-80]: signed PWM drive (ROS), ! heartbeat, front PSD blocks forward");
  out.println("# [V577,577]: wheel ticks/s (fwd +) PI drive; needs [Gff,static,kp,ki] first");
  out.println("# P/a: continuous PID/PSD, heartbeat required, max 10 sec");
  out.println("# q or space: STOP; z: STOP+zero counts; t: stream toggle; s: status");
  out.println("# 1/2/3: left30/60/90; 4/5/6: right30/60/90");
  out.println("# 7: start pose; 8: grip pose; [L105]/[R095]: angle 0~180, 3 digits (STOP only)");
}
void setup() {
  Serial.begin(115200); PiSerial.begin(115200);
  analogReadResolution(10); analogWriteResolution(8);
  pinMode(LEFT_PWM,OUTPUT); pinMode(LEFT_DIR,OUTPUT); pinMode(RIGHT_PWM,OUTPUT); pinMode(RIGHT_DIR,OUTPUT);
  stopMotors();
  pinMode(LEFT_ENC_A,INPUT_PULLUP); pinMode(LEFT_ENC_B,INPUT_PULLUP);
  pinMode(RIGHT_ENC_A,INPUT_PULLUP); pinMode(RIGHT_ENC_B,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),leftEncoderISR,RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A),rightEncoderISR,RISING);
  servoLeft.attach(D2); servoRight.attach(D3);
  servoLeft.write(SERVO_LEFT_START); servoRight.write(SERVO_RIGHT_START);
  lastServoTime=millis();
  say("# ROBOT40_V3 READY");
  say("ms,mode,L,R,PWM_L,PWM_R,PSD_FRONT,PSD_REAR,PSD_LEFT,PSD_RIGHT,SERVO_L,SERVO_R");
}
void loop() {
  for(int n=0;n<16 && PiSerial.available();++n) receiveByte(PiSerial.read(),parsers[0],0);
  for(int n=0;n<16 && Serial.available();++n) receiveByte(Serial.read(),parsers[1],1);
  updateServos();
  uint32_t now=millis();
  if(calibrating && now-calibrationTime>120000) { calibrating=false; say("# CAL_TIMEOUT"); }
  if(currentMode!=MODE_STOP) {
    if(needsHeartbeat() && now-heartbeatTime[motionOwner]>HEARTBEAT_TIMEOUT_MS) halt("# ABORT HEARTBEAT_TIMEOUT");
    else if(currentMode==MODE_DISTANCE && now-modeStartTime>=DISTANCE_TIMEOUT_MS) halt("# ABORT DISTANCE_TIMEOUT");
    else if((currentMode==MODE_PID_CONTINUOUS || currentMode==MODE_AUTO_PSD || wheelHold) && now-modeStartTime>=CONTINUOUS_LIMIT_MS) halt("# ABORT TEST_TIME_LIMIT");
    else if(psdHeld(currentMode!=MODE_AUTO_PSD && currentMode!=MODE_DRIVE && analogRead(PSD_FRONT)>=PSD_THRESHOLD,frontOverSince,now)) halt("# ABORT PSD_FRONT");
    else if(psdHeld((currentMode==MODE_LEFT_TEST || currentMode==MODE_RIGHT_TEST) &&
            (analogRead(PSD_REAR)>=PSD_THRESHOLD || analogRead(PSD_L)>=PSD_THRESHOLD || analogRead(PSD_R)>=PSD_THRESHOLD),pivotOverSince,now)) halt("# ABORT PSD_PIVOT");
  }
  if(currentMode==MODE_DISTANCE) checkDistance();
  switch(currentMode) {
    case MODE_LEFT_TEST: setMotors(BASE_PWM,0); break;
    case MODE_RIGHT_TEST: setMotors(0,BASE_PWM); break;
    case MODE_BOTH_TEST: setMotors(BASE_PWM,BASE_PWM); break;
    case MODE_PID_TEST: case MODE_PID_CONTINUOUS: case MODE_DISTANCE: runPID(); break;
    case MODE_AUTO_PSD: runPSDAuto(); break;
    case MODE_DRIVE: {
      // Front PSD blocks forward motion only, so the robot can still turn/back away.
      bool blocked=psdHeld(analogRead(PSD_FRONT)>=PSD_THRESHOLD,frontOverSince,now);
      bool forward=velMode?velTarget[0]+velTarget[1]>0:driveL+driveR>0;
      if(velMode) velocityControl(now);
      if(blocked && forward) { setMotors(0,0); velInt[0]=velInt[1]=0; }
      else if(velMode) setMotors(velPwm[0],velPwm[1]);
      else setMotors(driveL,driveR);
      break;
    }
    default: break;
  }
  if(!wheelHold && currentMode>=MODE_LEFT_TEST && currentMode<=MODE_PID_TEST && now-modeStartTime>=MOTOR_TEST_MS) halt("# DONE TIMED_TEST");
  updateMotors();
  if(currentMode!=MODE_STOP) checkStall();
  if(settling && now-settleStart>=SETTLE_MS) {
    settling=false; reportDistance(PiSerial); reportDistance(Serial); say("# DONE DISTANCE_400");
  }
  if(currentMode==MODE_STOP) {
    if(helpPending) { helpPending=false; help(PiSerial); help(Serial); }
    if(statusPending) {
      statusPending=false; say("# STATUS ROBOT40_V3");
      printTelemetry(PiSerial); printTelemetry(Serial);
      reportCalibration(PiSerial); reportCalibration(Serial);
    }
  }
  if((telemetryStream || currentMode!=MODE_STOP || settling || calibrating) && now-lastTelemetryTime>=TELEMETRY_MS) {
    lastTelemetryTime=now; printTelemetry(PiSerial); printTelemetry(Serial);
  }
}
