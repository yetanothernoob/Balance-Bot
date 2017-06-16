#include "MPU6050_6Axis_MotionApps20.h"
#include "I2Cdev.h"
#include "Wire.h"

MPU6050 mpu;
Quaternion q;
VectorFloat gravity;
float ypr[3];

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

double originalSetpoint = 180;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.3;
double input, output, p=70, i=240, d=1.9, ITerm, lastInput, error;
double outMin=-255, outMax=255, dInput;
double motorAConst=1, motorBConst=1;
int SampleTime = 10;
unsigned long lastTime = millis()-SampleTime;

int ENA = 3;
int IN1 = 4;
int IN2 = 8;
int IN3 = 5;
int IN4 = 7;
int ENB = 6;
int currentSpeed;
int MIN_ABS_SPEED = 30;

volatile bool mpuInterrupt = false;
void dmpDataReady(){
  mpuInterrupt = true;
}

void Compute(){
  unsigned long now = millis();
  unsigned long timeChange = now - lastTime;
  if(timeChange>=SampleTime){
    error = setpoint - input;
    ITerm = ITerm + (i*error);
    if(ITerm > outMax) ITerm = outMax;
    else if(ITerm < outMin) ITerm = outMin;
    dInput = (input - lastInput);
    output = p * error + ITerm- d * dInput;
    if(output > outMax) output = outMax;
    else if(output < outMin) output = outMin;
    lastInput = input;
    lastTime = now;
  }
}

void Move(int speed, int minSpeed){
  int direction = 1;
  if(speed < 0){
    direction = -1;
    speed = min(speed,-1*minSpeed);
    speed = max(speed,-255);
  }
  else
  {
    speed = max(speed,minSpeed);
    speed = min(speed,255);
  }
  if(speed == currentSpeed) return;
  int realSpeed = max(minSpeed, abs(speed));
  digitalWrite(IN1, speed > 0 ? HIGH : LOW);
  digitalWrite(IN2, speed > 0 ? LOW : HIGH);
  digitalWrite(IN3, speed > 0 ? HIGH : LOW);
  digitalWrite(IN4, speed > 0 ? LOW : HIGH);
  analogWrite(ENA, realSpeed * motorAConst);
  analogWrite(ENB, realSpeed * motorBConst);
  currentSpeed = direction * realSpeed;
}

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  Wire.begin();
  TWBR = 24;//jashgdjhbasdhja
  Serial.begin(115200);
  while(!Serial);
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  if (devStatus == 0){
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();  
  }
  else{
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void loop() {
  if (!dmpReady) return;
  while (!mpuInterrupt && fifoCount < packetSize){
    Compute();
    Move(output, MIN_ABS_SPEED);
  }
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024){
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  }
  else if (mpuIntStatus & 0x02){
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180/M_PI);
    //pitch = (1 - alpha)*pitch + alpha*ypr[1];
    //Serial.println(pitch * 180/M_PI);
    input = (ypr[1]) * 180/M_PI + 180;    
   }
}
