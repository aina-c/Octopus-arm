#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#define ENCA 4 // white 1
#define ENCB 5 // yellow 1
#define ENCC 3 // white 2
#define ENCD 2 // yellow 2

#define PWM1 10 //orange motor 1
#define PWM2 11 //yellow motor 2

#define IN22 6  //motor 2 in2
#define IN21 7  // motor2 in1
#define IN12 8  //motor1 in2
#define IN11 9  //motor1 in1

int sensorpin = A0;
int sensor;

volatile int posi1 = 0; 
volatile int posi2 = 0;

long prevT = 0;
float eprev1 = 0;
float eprev2 = 0;
float eintegral1 = 0;
float eintegral2 = 0;



void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder1,RISING);
  pinMode(ENCD,INPUT);
  pinMode(ENCC,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCC),readEncoder2,RISING);
  
  pinMode(PWM1,OUTPUT);
  pinMode(IN11,OUTPUT);
  pinMode(IN12,OUTPUT);

  pinMode(PWM2,OUTPUT);
  pinMode(IN21,OUTPUT);
  pinMode(IN22,OUTPUT);
  
  Serial.println("target pos");

}

void loop() {

  // set target position
  sensor = analogRead(sensorpin);
  Serial.println(sensor);
  int t1 = map(sensor, 0, 1022, 0, 2100);
  int t2 = map(sensor, 0, 1022, 0, 2100);

  goTarget(t1,t2);
  
  
}

void goTarget(int target1,int target2){
  // PID constants
  float kp = 1;
  float kd = 0.0;
  float ki = 0.0;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  
  int pos1 = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos1 = posi1;
  }
  int pos2 = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos2 = posi2;
  }
  
  // error
  int e1 = pos1 - target1;
  int e2 = pos2 - target2;

  // derivative
  float dedt1 = (e1-eprev1)/(deltaT);
  float dedt2 = (e2-eprev2)/(deltaT);

  // integral
  eintegral1 = eintegral1 + e1*deltaT;
  eintegral2 = eintegral2 + e2*deltaT;

  // control signal
  float u1 = kp*e1 + kd*dedt1 + ki*eintegral1;
  float u2 = kp*e2 + kd*dedt2 + ki*eintegral2;

  // motor power
  float pwr1 = fabs(u1);
  if( pwr1 > 255 ){
    pwr1 = 255;
  }
  float pwr2 = fabs(u2);
  if( pwr2 > 255 ){
    pwr2 = 255;
  }

  // motor direction
  int dir1 = 1;
  if(u1<0){
    dir1 = -1;
  }
  int dir2 = 1;
  if(u2<0){
    dir2 = -1;
  }

  // signal the motor
  setMotor(dir1,pwr1,PWM1,IN11,IN12);
  setMotor(dir2,pwr2,PWM2,IN21,IN22);


  // store previous error
  eprev1 = e1;
  eprev2 = e2;

  //prints
  Serial.print(pwr1);
  Serial.print(" ");
  Serial.print(dir1);
  Serial.print(" ");
  Serial.print(target1);
  Serial.print(" ");
  Serial.print(pos1);
  Serial.print("       ");
  Serial.print(pwr2);
  Serial.print(" ");
  Serial.print(dir2);
  Serial.print(" ");
  Serial.print(target2);
  Serial.print(" ");
  Serial.print(pos2);
  Serial.println();

}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  //power for motor
  analogWrite(pwm,pwmVal);

  //direction of spining
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

void readEncoder1(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi1++;
  }
  else{
    posi1--;
  }
}
void readEncoder2(){
  int b = digitalRead(ENCD);
  if(b > 0){
    posi2++;
  }
  else{
    posi2--;
  }
}