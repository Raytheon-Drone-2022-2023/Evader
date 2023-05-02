#include <PID_v1.h>
#include <util/atomic.h>

#define channelA 6 // in3 right motor
#define brakeA 7  // in4 right motor
#define channelB 4 // in1 left motor
#define brakeB 5 // in2 left motor
#define enA 10 //enA left motor
#define enB 9 //enB right motor

#define rightEncoderA 3
#define rightEncoderB 13

#define leftEncoderA 2
#define leftEncoderB 12


// A class to compute the control signal
class SimplePID{
  private:
    float kp, kd, ki, umax; // Parameters
    float eprev, eintegral; // Storage

  public:
  // Constructor
  SimplePID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0){}

  // A function to set the parameters
  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn){
    kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
  }

  // A function to compute the control signal
  void evalu(int value, int target, float deltaT, int &pwr, int &dir){
    // error
    int e = value - target;
  
    // derivative
    float dedt = (e-eprev)/(deltaT);
  
    // integral
    eintegral = eintegral + e*deltaT;
  
    // control signal
    float u = kp*e + kd*dedt + ki*eintegral;
  
    // motor power
    pwr = (int) fabs(u);
    if( pwr > umax ){
      pwr = umax;
    }
  
    // motor direction
    dir = 1;
    if(u<0){
      dir = -1;
    }
  
    // store previous error
    eprev = e;
  }
  
};

// How many motors
#define NMOTORS 2

// Pins
const int enca[] = {3,2};
const int encb[] = {13,12};
const int pwm[] = {9,10};
const int in1[] = {6,4};
const int in2[] = {7,5};
const int killSwitch = A1;
const int buzzer = A2;
const int hit = 8;

// Globals
long prevT = 0;
int posPrev[] = {0,0};
volatile int posi[] = {0,0};
float velocity[] = {0,0};
bool switchVal;


// PID class instances
SimplePID pid[NMOTORS];

void setup() {
  Serial.begin(9600);
  pinMode(killSwitch, INPUT);
  pinMode(hit, INPUT);
  pinMode(buzzer, OUTPUT);
  noTone(buzzer);
  for(int k = 0; k < NMOTORS; k++){
    pinMode(enca[k],INPUT);
    pinMode(encb[k],INPUT);
    pinMode(pwm[k],OUTPUT);
    pinMode(in1[k],OUTPUT);
    pinMode(in2[k],OUTPUT);
  }
  pid[0].setParams(1,0,0,255);
  pid[1].setParams(1,0,0,255);
  
  attachInterrupt(digitalPinToInterrupt(enca[0]),readEncoder<0>,FALLING);
  attachInterrupt(digitalPinToInterrupt(enca[1]),readEncoder<1>,RISING);
  
  Serial.println("target pos");
}

void loop() {
  switchVal = readSwitch(killSwitch, false);
  Serial.print(" Switch value: ");
  Serial.println(switchVal);
  if (switchVal){
    killEvader();
  }
  checkHit();
  // create a velocity target
  int target[NMOTORS];
  target[0] = 0; //750*sin(prevT/1e6);
  target[1] = 0;//100*sin(prevT/1e6);

  for(int k = 0; k < NMOTORS; k++){
    int pwr, dir = 0;
    // evaluate the control signal
    // the pos is replaced with velocity
    // signal the motor
    setMotor(dir,pwr,pwm[k],in1[k],in2[k]);
  }

  // Read the position in an atomic block to avoid a potential misread
  int pos[NMOTORS];
  float velocity[NMOTORS];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    for(int k = 0; k < NMOTORS; k++){
      pos[k] = posi[k];
    }
  }
  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  for(int k = 0; k < NMOTORS; k++){
    velocity[k] = (pos[k] - posPrev[k]) /deltaT;
    posPrev[k] = pos[k];
  }
  prevT = currT;

  //TODO: convert count/s to RPM (need motor specs)
  float v1 = 0;
  float v2 = 0;

  //TODO: going to need a Low-Pass filter
  
  // loop through the motors
  for(int k = 0; k < NMOTORS; k++){
    int pwr, dir;
    // dir = -1;
    // pwr = 100/3.0*micros()/1.0e6;
    // evaluate the control signal
    // the pos is replaced with velocity
    pid[k].evalu(velocity[k],target[k],deltaT,pwr,dir);
    // signal the motor
    setMotor(dir,pwr,pwm[k],in1[k],in2[k]);
  }
  Serial.print(target[0]);
  for(int k = 0; k < NMOTORS; k++){
    
    Serial.print(" ");
    Serial.print(velocity[k]);
  }
  // Serial.print("Target1:");
  //   Serial.print(target[1]);
  //   Serial.print(",");
  //   Serial.print(" ");
    // Serial.print("Velocity1:");
    // Serial.print(velocity[1]);
    // Serial.print(" ");
  Serial.println();
  delay(500);
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

template <int j>
void readEncoder(){
  int b = digitalRead(encb[j]);
  if(b > 0){
    posi[j]++;
  }
  else{
    posi[j]--;
  }
}

int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue){
  int ch = pulseIn(channelInput, HIGH, 30000);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}
bool readSwitch(byte channelInput, bool defaultValue){
  int intDefaultValue = (defaultValue)? 100: 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}
void killEvader(){
  for(int k = 0; k < NMOTORS; k++){
    int pwr, dir = 0;
    // evaluate the control signal
    // the pos is replaced with velocity
    // signal the motor
    setMotor(dir,pwr,pwm[k],in1[k],in2[k]);
  }
  while(true){}
}
void checkHit(){
  int b = digitalRead(hit);
  Serial.print(" Hit Value: ");
  Serial.println(b);
  if (b == 1){
    Serial.println("I am hit!");
    evaderHit();
  }  
}
void evaderHit(){
    for(int k = 0; k < NMOTORS; k++){
    int pwr, dir = 0;
    // evaluate the control signal
    // the pos is replaced with velocity
    // signal the motor
    setMotor(dir,pwr,pwm[k],in1[k],in2[k]);
  }
  tone(buzzer, 500, 4000);
  while(true){}
}