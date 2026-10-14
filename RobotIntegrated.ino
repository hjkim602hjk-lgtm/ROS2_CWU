// NUCLEO-F446RE / STM32duino 3.0.0. BASE_NAV_V1 (2026-10-14).
// Calibrated bench modes + experimental wheel velocity PI for ROS2 bridge.
// USB 400mm test is bounded, but USB disconnection is NOT detected.
// Encoder counting: A-channel RISING only. Calibrate THIS firmware's counts.
#include <Arduino.h>
#include <Servo.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
constexpr int SERVO_LEFT_START=40, SERVO_RIGHT_START=140;
constexpr int SERVO_LEFT_GRIP=105, SERVO_RIGHT_GRIP=75;
constexpr int SERVO_LEFT_MIN=0, SERVO_LEFT_MAX=180;
constexpr int SERVO_RIGHT_MIN=0, SERVO_RIGHT_MAX=180;

Uart PiSerial(PC_11, PC_10); // RX, TX
constexpr uint32_t LEFT_PWM=D6, LEFT_DIR=D7, RIGHT_DIR=D8, RIGHT_PWM=PA7_ALT3;
constexpr uint32_t LEFT_ENC_A=D4, LEFT_ENC_B=D5, RIGHT_ENC_A=D9, RIGHT_ENC_B=D10;
constexpr uint32_t PSD_FRONT=A0, PSD_REAR=A1, PSD_R=A2, PSD_L=A3;
constexpr bool INVERT_LEFT_MOTOR=true, INVERT_RIGHT_MOTOR=false;
constexpr int LEFT_ENCODER_SIGN=1, RIGHT_ENCODER_SIGN=-1;
constexpr int BASE_PWM=80, MAX_PWM=130, TURN_PWM=90, PSD_THRESHOLD=550;
constexpr float Kp=0.90f, Ki=0.40f, Kd=0.0f, MAX_CORRECTION=30.0f;
constexpr uint32_t CONTROL_MS=50, MOTOR_TEST_MS=10000, TELEMETRY_MS=200;
constexpr uint32_t HEARTBEAT_TIMEOUT_MS=700, ENCODER_TIMEOUT_MS=350;
constexpr uint32_t DISTANCE_TIMEOUT_MS=15000, CONTINUOUS_LIMIT_MS=10000;
constexpr uint32_t SETTLE_MS=300, SERVO_STEP_MS=30;
constexpr float MAX_WHEEL_MISMATCH_MM=40.0f;
constexpr float PHYSICAL_TRACK_MM=206.0f, TRACK_MM=190.4f;
float distanceTargetMM=1000.0f, turnTargetDeg=90.0f;
constexpr uint32_t VELOCITY_TIMEOUT_MS=300;
// Initial values only: NOT the experimentally tuned straight-sync PID gains.
constexpr float SPEED_KP=0.25f, SPEED_KI=0.60f, SPEED_FF=0.45f, SPEED_STATIC=35.0f;
constexpr float MAX_V_MM_S=150.0f, MAX_W_RAD_S=1.2f, MAX_WHEEL_MM_S=180.0f;
bool navArmed=false, navStream=false;
float velocityL=0,velocityR=0,speedIL=0,speedIR=0;
int speedSignL=0,speedSignR=0;
uint32_t velocityTime=0;
// Empirical distance calibration validated with 400 mm and 1000 mm trials.
float countsPerMmL=(5687.0f/1460.0f)*(40.0f/26.0f);
float countsPerMmR=(5600.0f/1460.0f)*(40.0f/26.0f);

enum RunMode { MODE_STOP=0, MODE_LEFT_TEST=1, MODE_RIGHT_TEST=2,
  MODE_BOTH_TEST=3, MODE_PID_TEST=4, MODE_PID_CONTINUOUS=5,
  MODE_AUTO_PSD=6, MODE_DISTANCE=7, MODE_TURN_LEFT=8, MODE_TURN_RIGHT=9, MODE_VELOCITY=10 };
RunMode completedMode=MODE_STOP;
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
uint32_t heartbeatTime[2]={0,0}; bool heartbeatSeen[2]={false,false};
uint8_t motionOwner=0, calibrationOwner=0;
int32_t startL=0, startR=0, previousL=0, previousR=0, watchedL=0, watchedR=0;
float integral=0, previousError=0;
int wrongCycles=0;
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
  navArmed=false; velocityL=velocityR=speedIL=speedIR=0;
  stopMotors(); currentMode=MODE_STOP; settling=false;
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
bool positioned() { return currentMode==MODE_DISTANCE || currentMode==MODE_TURN_LEFT || currentMode==MODE_TURN_RIGHT; }
bool turning() { return currentMode==MODE_TURN_LEFT || currentMode==MODE_TURN_RIGHT; }
int directionL() { return currentMode==MODE_TURN_LEFT?-1:1; }
int directionR() { return currentMode==MODE_TURN_RIGHT?-1:1; }
bool needsHeartbeat() {
  return currentMode!=MODE_VELOCITY && (motionOwner==0 || currentMode==MODE_PID_CONTINUOUS);
}
bool startMode(RunMode mode,uint8_t source) {
  if(currentMode!=MODE_STOP || settling || calibrating || servoBusy()) { say("# REJECT BUSY"); return false; }
  if((mode==MODE_DISTANCE || mode==MODE_TURN_LEFT || mode==MODE_TURN_RIGHT || mode==MODE_VELOCITY) && !calibrated()) { say("# REJECT CALIBRATE_FIRST"); return false; }
  if(mode!=MODE_VELOCITY && (source==0 || mode==MODE_PID_CONTINUOUS)) {
    if(!heartbeatSeen[source] || millis()-heartbeatTime[source]>HEARTBEAT_TIMEOUT_MS) {
      say("# REJECT HEARTBEAT_REQUIRED"); return false;
    }
  }
  stopMotors(); motionOwner=source;
  if(mode!=MODE_VELOCITY) navArmed=false;
  speedIL=speedIR=0; speedSignL=speedSignR=0;
  snapshot(startL,startR); previousL=watchedL=startL; previousR=watchedR=startR;
  integral=previousError=0; wrongCycles=0; distanceL=distanceR=0;
  reachedL=reachedR=false;
  modeStartTime=lastControlTime=lastPulseL=lastPulseR=millis();
  currentMode=mode; say("# START"); return true;
}
void reportDistance(Stream &out) {
  int32_t l,r; snapshot(l,r);
  bool turn=completedMode==MODE_TURN_LEFT || completedMode==MODE_TURN_RIGHT;
  float dl=(float)(l-startL)/countsPerMmL,dr=(float)(r-startR)/countsPerMmR;
  if(turn) {
    if(completedMode==MODE_TURN_LEFT) dl=-dl; else dr=-dr;
    out.print("# TURN_MM,"); out.print(dl,2); out.print(','); out.println(dr,2);
    out.print("# TURN_EST_DEG,"); out.println((dl+dr)/TRACK_MM*180.0f/PI,2);
  } else { out.print("# DIST_MM,"); out.print(dl,2); out.print(','); out.println(dr,2); }
}
void checkDistance() {
  int32_t l,r; snapshot(l,r);
  distanceL=directionL()*(float)(l-startL)/countsPerMmL;
  distanceR=directionR()*(float)(r-startR)/countsPerMmR;
  if(distanceL<-3 || distanceR<-3) { halt("# ABORT ENCODER_DIRECTION"); return; }
  if(fabsf(distanceL-distanceR)>MAX_WHEEL_MISMATCH_MM) { halt("# ABORT WHEEL_MISMATCH"); return; }
  reachedL=reachedL || distanceL>=distanceTargetMM;
  reachedR=reachedR || distanceR>=distanceTargetMM;
  if(reachedL) { analogWrite(LEFT_PWM,0); currentLeftPWM=targetLeftPWM=0; }
  if(reachedR) { analogWrite(RIGHT_PWM,0); currentRightPWM=targetRightPWM=0; }
  if(reachedL && reachedR) {
    completedMode=currentMode;
    stopMotors(); currentMode=MODE_STOP; settling=true; settleStart=millis();
    say("# TARGET_REACHED");
  }
}
void runPID() {
  uint32_t now=millis(),elapsed=now-lastControlTime;
  if(elapsed<CONTROL_MS) return;
  int32_t l,r; snapshot(l,r);
  float dl=directionL()*(float)(l-previousL),dr=directionR()*(float)(r-previousR);
  previousL=l; previousR=r; lastControlTime=now;
  float rightEquivalent=dr;
  if(positioned()) rightEquivalent*=countsPerMmL/countsPerMmR;
  float error=(dl-rightEquivalent)*CONTROL_MS/elapsed;
  float dt=elapsed/1000.0f;
  float correction;
  if(turning()) correction=constrain(0.60f*error+0.04f*(distanceL-distanceR)*countsPerMmL,-15.0f,15.0f);
  else {
    integral=constrain(integral+error*dt,-100.0f,100.0f);
    correction=constrain(Kp*error+Ki*integral+Kd*(error-previousError)/dt,-MAX_CORRECTION,MAX_CORRECTION);
  }
  previousError=error;
  int base=turning()?75:BASE_PWM;
  if(positioned()) {
    float remaining=distanceTargetMM-min(distanceL,distanceR);
    if(remaining<(turning()?30.0f:60.0f)) base=turning()?65:70;
    if(reachedL || reachedR) correction=0;
  }
  int trim=(int)lroundf(correction);
  int pl=constrain(base-trim,0,MAX_PWM),pr=constrain(base+trim,0,MAX_PWM);
  if(positioned()) { if(reachedL) pl=0; if(reachedR) pr=0; }
  setMotors(directionL()*pl,directionR()*pr);
  if(dl<-1 || dr<-1) ++wrongCycles; else wrongCycles=0;
  if(wrongCycles>=3) halt("# ABORT ENCODER_DIRECTION");
}
int speedPWM(float target,float measured,float dt,float &acc,int &previousSign) {
  int sign=(target>0)-(target<0);
  if(sign==0) { acc=0; previousSign=0; return 0; }
  if(sign!=previousSign) { acc=0; previousSign=sign; }
  float error=fabsf(target)-measured*sign;
  float candidate=constrain(acc+SPEED_KI*error*dt,-40.0f,40.0f);
  float ff=SPEED_STATIC+SPEED_FF*fabsf(target);
  float u=ff+SPEED_KP*error+candidate;
  if((u>=0 && u<=MAX_PWM) || (u>MAX_PWM && error<0) || (u<0 && error>0)) acc=candidate;
  return sign*(int)lroundf(constrain(ff+SPEED_KP*error+acc,0.0f,(float)MAX_PWM));
}
void runVelocity() {
  uint32_t now=millis(),elapsed=now-lastControlTime;
  if(elapsed<CONTROL_MS) return;
  if(elapsed>200) { halt("# ABORT CONTROL_OVERRUN"); return; }
  int32_t l,r; snapshot(l,r);
  float dt=elapsed/1000.0f;
  float vl=(l-previousL)/countsPerMmL/dt, vr=(r-previousR)/countsPerMmR/dt;
  previousL=l; previousR=r; lastControlTime=now;
  setMotors(speedPWM(velocityL,vl,dt,speedIL,speedSignL),speedPWM(velocityR,vr,dt,speedIR,speedSignR));
  bool wrong=(fabsf(velocityL)>20 && vl*velocityL<0 && fabsf(vl)>10) ||
             (fabsf(velocityR)>20 && vr*velocityR<0 && fabsf(vr)>10);
  if(wrong) ++wrongCycles; else wrongCycles=0;
  if(wrongCycles>=3) halt("# ABORT ENCODER_DIRECTION");
}
bool blockedPSD() {
  int f=analogRead(PSD_FRONT),b=analogRead(PSD_REAR),l=analogRead(PSD_L),r=analogRead(PSD_R);
  if(currentMode==MODE_VELOCITY) {
    bool front=velocityL>0 || velocityR>0 || currentLeftPWM>0 || currentRightPWM>0;
    bool rear=velocityL<0 || velocityR<0 || currentLeftPWM<0 || currentRightPWM<0;
    bool rotate=fabsf(velocityL-velocityR)>1 || (currentLeftPWM*currentRightPWM<0);
    return (front && f>=PSD_THRESHOLD) || (rear && b>=PSD_THRESHOLD) ||
           (rotate && (l>=PSD_THRESHOLD || r>=PSD_THRESHOLD));
  }
  if(turning() || currentMode==MODE_LEFT_TEST || currentMode==MODE_RIGHT_TEST)
    return f>=PSD_THRESHOLD || b>=PSD_THRESHOLD || l>=PSD_THRESHOLD || r>=PSD_THRESHOLD;
  return f>=PSD_THRESHOLD;
}
void checkStall() {
  int32_t l,r; snapshot(l,r); uint32_t now=millis();
  if(currentMode==MODE_VELOCITY && now-modeStartTime<700) { lastPulseL=lastPulseR=now; }
  if(l!=watchedL || currentLeftPWM==0) lastPulseL=now;
  if(r!=watchedR || currentRightPWM==0) lastPulseR=now;
  watchedL=l; watchedR=r;
  if((currentLeftPWM!=0 && now-lastPulseL>=ENCODER_TIMEOUT_MS) ||
     (currentRightPWM!=0 && now-lastPulseR>=ENCODER_TIMEOUT_MS)) halt("# ABORT ENCODER_NO_PULSE");
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
void frameCommand(char *s,uint8_t source) {
  if(strcmp(s,"N1")==0) { navStream=true; say("# NAV_STREAM_ON"); return; }
  if(strcmp(s,"N0")==0) { navStream=false; say("# NAV_STREAM_OFF"); return; }
  if(strcmp(s,"ARM")==0) {
    if(source!=0 || currentMode!=MODE_STOP || settling || calibrating || servoBusy()) { say("# REJECT ARM_STATE"); return; }
    navArmed=true; velocityTime=millis(); say("# ARMED"); return;
  }
  if(s[0]=='V') {
    char *end; float v=strtof(s+1,&end);
    if(end==s+1 || *end!=',') { halt("# ABORT VELOCITY_FORMAT"); return; }
    char *next=end+1; float w=strtof(next,&end);
    if(end==next || *end || !isfinite(v) || !isfinite(w) || fabsf(v)>MAX_V_MM_S || fabsf(w)>MAX_W_RAD_S) {
      halt("# ABORT VELOCITY_RANGE"); return;
    }
    if(source!=0) { say("# REJECT VELOCITY_PI_ONLY"); return; }
    if(v==0 && w==0) {
      if(currentMode==MODE_VELOCITY) {
        stopMotors(); currentMode=MODE_STOP; speedIL=speedIR=0;
      }
      velocityTime=millis(); return;
    }
    if(!navArmed) { say("# REJECT ARM_REQUIRED"); return; }
    if(currentMode!=MODE_STOP && currentMode!=MODE_VELOCITY) { say("# REJECT BUSY"); return; }
    if(currentMode==MODE_STOP && !startMode(MODE_VELOCITY,source)) return;
    float l=v-w*TRACK_MM/2.0f,r=v+w*TRACK_MM/2.0f;
    float peak=max(fabsf(l),fabsf(r));
    if(peak>MAX_WHEEL_MM_S) { l*=MAX_WHEEL_MM_S/peak; r*=MAX_WHEEL_MM_S/peak; }
    // Stop before a sign reversal; next speed update starts from zero output.
    if(l*velocityL<0 || r*velocityR<0) {
      stopMotors(); speedIL=speedIR=0; wrongCycles=0;
      snapshot(previousL,previousR); lastControlTime=modeStartTime=millis();
    }
    velocityL=l; velocityR=r; velocityTime=millis(); return;
  }
  if(currentMode!=MODE_STOP || settling || servoBusy()) { say("# REJECT BUSY"); return; }
  if(strcmp(s,"ML")==0) { startMode(MODE_LEFT_TEST,source); return; }
  if(strcmp(s,"MR")==0) { startMode(MODE_RIGHT_TEST,source); return; }
  if(strcmp(s,"AUTO")==0) { say("# REJECT USE_PI_NAVIGATION"); return; }
  if(s[0]=='F' || (s[0]=='T' && (s[1]=='L' || s[1]=='R'))) {
    bool turn=s[0]=='T'; char *first=s+(turn?2:1),*end;
    float target=strtof(first,&end);
    if(end==first || *end || !isfinite(target) || target<(turn?10.0f:100.0f) || target>(turn?180.0f:1000.0f)) { say("# REJECT TARGET_RANGE"); return; }
    RunMode mode=turn?(s[1]=='L'?MODE_TURN_LEFT:MODE_TURN_RIGHT):MODE_DISTANCE;
    if(startMode(mode,source)) {
      turnTargetDeg=turn?target:0;
      distanceTargetMM=turn?PI*TRACK_MM*target/360.0f:target;
    }
    return;
  }
  if(strcmp(s,"CAL")==0) {
    if(calibrating) { say("# REJECT CAL_ACTIVE"); return; }
    stopMotors(); snapshot(calibrationL,calibrationR); calibrating=true;
    calibrationOwner=source; calibrationTime=millis();
    say("# CAL_BEGIN PUSH_FORWARD_400MM_THEN_[C400]"); return;
  }
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
  if(c=='b') { startMode(MODE_BOTH_TEST,source); return; }
  if(c=='p') { startMode(MODE_PID_TEST,source); return; }
  if(c=='P') { startMode(MODE_PID_CONTINUOUS,source); return; }
  if(c=='f') { if(startMode(MODE_DISTANCE,source)) distanceTargetMM=1000.0f; return; }
  if(c=='z') {
    calibrating=false; halt("# STOP");
    noInterrupts(); leftEncoderCount=rightEncoderCount=0; interrupts();
    say("# ENCODERS_RESET"); return;
  }
  if(currentMode!=MODE_STOP || settling || calibrating) { say("# REJECT BUSY"); return; }
  switch(c) {
    case 'a': servoLeftTarget=constrain(servoLeftTarget-1,0,180); break;
    case 'd': servoLeftTarget=constrain(servoLeftTarget+1,0,180); break;
    case 'A': servoLeftTarget=constrain(servoLeftTarget-5,0,180); break;
    case 'D': servoLeftTarget=constrain(servoLeftTarget+5,0,180); break;
    case 'j': servoRightTarget=constrain(servoRightTarget-1,0,180); break;
    case 'l': servoRightTarget=constrain(servoRightTarget+1,0,180); break;
    case 'J': servoRightTarget=constrain(servoRightTarget-5,0,180); break;
    case 'L': servoRightTarget=constrain(servoRightTarget+5,0,180); break;
    case 'c':
    case '7': servoLeftTarget=SERVO_LEFT_START; servoRightTarget=SERVO_RIGHT_START; break;
    case '8': servoLeftTarget=SERVO_LEFT_GRIP; servoRightTarget=SERVO_RIGHT_GRIP; break;
    default: say("# REJECT COMMAND"); return;
  }
  statusPending=true;
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
  out.println("# ROBOT40_V4"); out.println("# BASE_NAV_V1");
  out.println("# [F400]/[F1000]: forward mm; f:1000mm; [TL90]/[TR90]: degrees");
  out.println("# p/b/[ML]/[MR]:10sec test. Pi bench modes require ! every200ms");
  out.println("# [ARM] then [V100,0] : mm/s, rad/s; positive w=left(CCW)");
  out.println("# Velocity commands >=10Hz; timeout300ms disarms. ! cannot extend velocity command");
  out.println("# Velocity PI gains are initial/unvalidated. q stops and disarms");
  out.println("# [N1]/[N0]:20Hz telemetry on/off; s:status; h:help; z:stop+zero");
  out.println("# [Kleft,right]:counts/mm RAM; 7/c:open40/140; 8:grip105/75");
  out.println("# [L105]/[R075]:servo angle; [AUTO] disabled; navigation belongs on Pi");
  out.print("# TRACK_MM,"); out.println(TRACK_MM,3);
  reportCalibration(out);
}
void setup() {
  Serial.begin(115200); PiSerial.begin(115200);
  analogReadResolution(10); analogWriteResolution(8);
  pinMode(LEFT_PWM,OUTPUT); pinMode(LEFT_DIR,OUTPUT); pinMode(RIGHT_PWM,OUTPUT); pinMode(RIGHT_DIR,OUTPUT);
  digitalWrite(LEFT_DIR,LOW); digitalWrite(RIGHT_DIR,LOW);
  stopMotors();
  pinMode(PSD_FRONT,INPUT_ANALOG); pinMode(PSD_REAR,INPUT_ANALOG);
  pinMode(PSD_R,INPUT_ANALOG); pinMode(PSD_L,INPUT_ANALOG);
  pinMode(LEFT_ENC_A,INPUT_PULLUP); pinMode(LEFT_ENC_B,INPUT_PULLUP);
  pinMode(RIGHT_ENC_A,INPUT_PULLUP); pinMode(RIGHT_ENC_B,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),leftEncoderISR,RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A),rightEncoderISR,RISING);
  servoLeft.attach(D2); servoRight.attach(D3);
  servoLeft.write(SERVO_LEFT_START); servoRight.write(SERVO_RIGHT_START);
  lastServoTime=millis();
  say("# ROBOT40_V4 READY"); say("# BASE_NAV_V1");
  say("ms,mode,L,R,PWM_L,PWM_R,PSD_FRONT,PSD_REAR,PSD_LEFT,PSD_RIGHT,SERVO_L,SERVO_R");
}
void loop() {
  for(int n=0;n<16 && PiSerial.available();++n) receiveByte(PiSerial.read(),parsers[0],0);
  for(int n=0;n<16 && Serial.available();++n) receiveByte(Serial.read(),parsers[1],1);
  updateServos();
  uint32_t now=millis();
  if(calibrating && now-calibrationTime>120000) { calibrating=false; say("# CAL_TIMEOUT"); }
  if(currentMode!=MODE_STOP) {
    if(currentMode==MODE_VELOCITY && now-velocityTime>VELOCITY_TIMEOUT_MS) halt("# ABORT VELOCITY_TIMEOUT");
    else if(needsHeartbeat() && now-heartbeatTime[motionOwner]>HEARTBEAT_TIMEOUT_MS) halt("# ABORT HEARTBEAT_TIMEOUT");
    else if(positioned() && now-modeStartTime>=DISTANCE_TIMEOUT_MS) halt("# ABORT DISTANCE_TIMEOUT");
    else if(currentMode==MODE_PID_CONTINUOUS && now-modeStartTime>=CONTINUOUS_LIMIT_MS) halt("# ABORT TEST_TIME_LIMIT");
    else if(blockedPSD()) halt("# ABORT PSD_BLOCKED");
  }
  if(positioned()) checkDistance();
  switch(currentMode) {
    case MODE_LEFT_TEST: setMotors(BASE_PWM,0); break;
    case MODE_RIGHT_TEST: setMotors(0,BASE_PWM); break;
    case MODE_BOTH_TEST: setMotors(BASE_PWM,BASE_PWM); break;
    case MODE_PID_TEST: case MODE_PID_CONTINUOUS: case MODE_DISTANCE:
    case MODE_TURN_LEFT: case MODE_TURN_RIGHT: runPID(); break;
    case MODE_VELOCITY: runVelocity(); break;
    
    default: break;
  }
  if(currentMode>=MODE_LEFT_TEST && currentMode<=MODE_PID_TEST && now-modeStartTime>=MOTOR_TEST_MS) halt("# DONE TIMED_TEST");
  updateMotors();
  if(currentMode!=MODE_STOP) checkStall();
  if(settling && now-settleStart>=SETTLE_MS) {
    settling=false; reportDistance(PiSerial); reportDistance(Serial);
    if(completedMode==MODE_TURN_LEFT) say("# DONE TURN_LEFT");
    else if(completedMode==MODE_TURN_RIGHT) say("# DONE TURN_RIGHT");
    else if(fabsf(distanceTargetMM-400)<0.01f) say("# DONE DISTANCE_400");
    else if(fabsf(distanceTargetMM-1000)<0.01f) say("# DONE DISTANCE_1000");
    else say("# DONE DISTANCE");
  }
  if(currentMode==MODE_STOP) {
    if(helpPending) { helpPending=false; help(PiSerial); help(Serial); }
    if(statusPending) {
      statusPending=false; say("# STATUS ROBOT40_V4");
      printTelemetry(PiSerial); printTelemetry(Serial);
      reportCalibration(PiSerial); reportCalibration(Serial);
      Serial.print("# TARGET_L_R,"); Serial.print(servoLeftTarget); Serial.print(','); Serial.println(servoRightTarget);
      PiSerial.print("# TARGET_L_R,"); PiSerial.print(servoLeftTarget); PiSerial.print(','); PiSerial.println(servoRightTarget);
    }
  }
  if((navStream || telemetryStream || currentMode!=MODE_STOP || settling || calibrating) && now-lastTelemetryTime>=(navStream?50:TELEMETRY_MS)) {
    lastTelemetryTime=now; printTelemetry(PiSerial); printTelemetry(Serial);
  }
}
