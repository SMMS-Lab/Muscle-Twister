#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#include "AccelStepper.h" 
///////////////////////// Motor Speed arrays for [DC_motor, stepper_motor, stepper_returning_speed]
const float twisting_speed[3]       = {1, -500, 0};
const float coiling_speed[3]        = {1, 0, 0};
const float mandreling_speed[3]     = {0, 0, 1000.0}; 
const float carriageReturn_speed[3] = {0, 4000.0, 0};

///////////////////////////////////////////////////////////////////////////////////////////////////
// Assign Motor speeds to a single 2D array
float (*task_array[]) = {
  twisting_speed,
  coiling_speed,
  mandreling_speed,
  carriageReturn_speed
};
// DC encoder Motor  - 19:1 Metal Gearmotor 37Dx68L mm 12V with 64 CPR Encoder
// DC Controller     - TB67H420FTG Dual/Single Motor Driver Carrier
// Stepper Motors    - Nema 17
// Stepper Controller- A4988
// DC Motor Pins
#define ENCA 2
#define ENCB 3
#define IN1 4
#define IN2 5
#define PWM 6

// Stepper Motor Pins
#define dirPin1 8
#define stepPin1 9
#define dirPin2 10
#define stepPin2 11

// Limit Switch pin
#define limitSwitch 13


int task=0, turns_ = 0;
float pulsesPerTurn = 16*18.75, target_f = 0;
int posi = 0; 
long prevT = 0, currT = 0, target = 0;
float eprev = 0;
float eintegral = 0;
int crev = 300;
int pos = 0;

#define motorInterfaceType 1
AccelStepper wrappingStepper = AccelStepper(motorInterfaceType, stepPin1, dirPin1); //wind copper wire
AccelStepper mandrelStepper  = AccelStepper(motorInterfaceType, stepPin2, dirPin2); //mandrel wrapping

long initial_homing=-1;  // Used to Home Stepper at startup


void setup() {
  setMotor(0, 0, PWM, IN1, IN2);
  wrappingStepper.setMaxSpeed(1000.0);
  wrappingStepper.setAcceleration(100.0);
  mandrelStepper.setMaxSpeed(1000.0);
  mandrelStepper.setAcceleration(100.0);
  pinMode(limitSwitch,INPUT);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  Serial.begin(9600);
  carriageReset();
}

void loop() {
  Serial.println("Select the operation:\n\t1-Twisting\n\t2-Coiling\n\t3-Mandrel Coiling");

  while (Serial.available()) {
      Serial.read();
   }
  while(Serial.available() == 0){}
  task = Serial.parseInt();
  pos = 0;
 
  Serial.println("How many turns induced in the fishing line?");
  while (Serial.available()) {
      Serial.read();
   }
  while(Serial.available() == 0){}
  turns_ = Serial.parseInt(); //read in number of revolutions 
  
  while(pos<turns_){
    currT = micros(); // time difference
    pos = DCMotor();
    wrappingStepper.setSpeed(task_array[task-1][1]);
    wrappingStepper.setAcceleration(100.0);
    wrappingStepper.runSpeed();
    Serial.print(" ");
    Serial.print(turns_);
    Serial.print(" ");
    Serial.print(pos);
    Serial.println();
  }
  setMotor(0, 0, PWM, IN1, IN2);
}



//---------------------Functions---------------//
void setTarget(float t, float deltat){
  float positionChange = 0;
  t = fmod(t,12); // time is in seconds
  float velocity = task_array[task-1][0]; // rps
  if(target < turns_*pulsesPerTurn && turns_>0 ){
    positionChange = velocity*deltat*pulsesPerTurn;
  }
  else if(target > turns_*pulsesPerTurn && turns_< 0 ){
    positionChange = -velocity*deltat*pulsesPerTurn;
  }
  else {
    positionChange = 0;
  }
  target_f = target_f + positionChange; 
  target = (long)target_f;
}

int DCMotor(){
  float kp = 1.0;
  float kd = 0.015;
  float ki = 0.050;

  float deltaT = ((float) (currT - prevT)) / ( 1.0e6 );
  prevT = currT;

  setTarget(prevT/1.0e6,deltaT);

//  float velocity = task_array[task-1][0];
//  if(prevT/1.0e6*velocity>turns_){
//    target = target+ (long)(velocity*deltaT*pulsesPerTurn);
//  }
//  else if (prevT/1.0e6*velocity<turns_){
//    target = target- (long)(velocity*deltaT*pulsesPerTurn);
//  }
//  else {
//    target = target;
//  }
  

  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }
  long int e = pos - target; // error
  float dedt = (e - eprev) / (deltaT); // derivative
  eintegral = eintegral + e * deltaT; // integral
  float u = kp * e + kd * dedt + ki * eintegral; // control signal
  float pwr = fabs(u); // motor power
  if ( pwr > 255) { 
    pwr = 255;
  }
  int dir = 1; // motor direction
  if (u < 0) {
    dir = -1;
  }
  setMotor(dir, pwr, PWM, IN1, IN2);// signal the motor
  eprev = e; // store previous error
  return pos/crev;
}



void carriageReset(){
  Serial.print("Carriage is Homing . . . . . . . . . . . ");
  while (digitalRead(limitSwitch)) {  // Make the Stepper move CCW until the switch is activated   
    wrappingStepper.moveTo(initial_homing);  // Set the position to move to
    initial_homing++;  // Decrease by 1 for next move if needed
    wrappingStepper.run();  // Start moving the stepper
    delay(5);
}

  wrappingStepper.setCurrentPosition(0);  // Set the current position as zero for now
  wrappingStepper.setMaxSpeed(task_array[3][1]);      // Set Max Speed of Stepper (Slower to get better accuracy)
  wrappingStepper.setAcceleration(100.0);  // Set Acceleration of Stepper
  initial_homing=1;

  while (!digitalRead(limitSwitch)) { // Make the Stepper move CW until the switch is deactivated
    wrappingStepper.moveTo(initial_homing);  
    wrappingStepper.run();
    initial_homing--;
    delay(5);
  }
  
  wrappingStepper.setCurrentPosition(0);
  Serial.println("Homing Completed");
  wrappingStepper.setMaxSpeed(task_array[3][1]);
  wrappingStepper.setAcceleration(100.0);
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}


void readEncoder() {
  int b = digitalRead(ENCB);
  if (b > 0) {
    posi++;
  }
  else {
    posi--;
  }
}
